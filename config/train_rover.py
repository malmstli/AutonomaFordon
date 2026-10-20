"""ML-Agents CLI with fail-closed checkpoint transfer for the final rover."""
import os
import torch
from mlagents_envs.environment import UnityEnvironment
from mlagents.trainers.model_saver.torch_model_saver import TorchModelSaver
from mlagents.trainers.learn import main
from mlagents.trainers.trainer_controller import TrainerController

torch.set_num_threads(1)
UnityEnvironment.DEFAULT_EDITOR_PORT = int(os.environ.get('ROVER_EDITOR_PORT', '5004'))
original_load = TorchModelSaver._load_model
original_advance = TrainerController.advance
original_save = TorchModelSaver.save_checkpoint
FIRST_WEIGHT = 'network_body._body_endoder.seq_layers.0.weight'

def freeze_navigation(actor, optimizer):
    if hasattr(actor, '_rover_navigation_reference'): return
    actor._rover_navigation_reference = {k: v.detach().clone() for k, v in actor.state_dict().items()}
    for parameter in actor.parameters(): parameter.requires_grad_(False)
    weight = dict(actor.named_parameters())[FIRST_WEIGHT]
    weight.requires_grad_(True)
    actor.zero_grad(set_to_none=True)
    def sensor_gradient(gradient):
        result = gradient.clone()
        result[:, :11] = 0
        return result
    weight.register_hook(sensor_gradient)
    # Zero old Adam momentum too; zero gradients alone do not freeze columns.
    for value in optimizer.state.get(weight, {}).values():
        if torch.is_tensor(value) and value.shape == weight.shape: value[:, :11] = 0
    print('ROVER_NAVIGATION_FROZEN: PPO trains sensor columns and critic; empty-sensor navigation is preserved.', flush=True)

def verify_navigation(actor):
    reference = getattr(actor, '_rover_navigation_reference', None)
    if reference is None: return
    for name, actual in actor.state_dict().items():
        expected = reference[name]
        if name == FIRST_WEIGHT: actual, expected = actual[:, :11], expected[:, :11]
        if not torch.equal(actual, expected): raise RuntimeError('Frozen navigation changed: ' + name)

def guarded_advance(self, env_manager):
    sampler = self.param_manager.get_current_samplers().get('rover_stage')
    if sampler is not None and getattr(sampler, 'value', 0) >= 2:
        for trainer in self.trainers.values():
            if getattr(trainer, 'policy', None) is not None:
                if trainer.shared_critic: raise RuntimeError('Final rover requires PPO separate critic.')
                freeze_navigation(trainer.policy.actor, trainer.optimizer.optimizer)
    return original_advance(self, env_manager)

def guarded_save(self, behavior_name, step):
    verify_navigation(self.policy.actor)
    return original_save(self, behavior_name, step)

def checked_load(self, load_path, policy=None, reset_global_steps=False):
    saved = torch.load(load_path, map_location='cpu')
    weight = saved['Policy']['network_body._body_endoder.seq_layers.0.weight']
    if tuple(weight.shape) != (128, 21) or saved['Policy']['continuous_act_size_vector'].item() != 2:
        raise RuntimeError('Checkpoint is not a final 21-input, 2-action policy. Run prepare_final_seed.py first.')
    modules = self.modules if policy is None else policy.get_modules()
    for name in ('Policy', 'Optimizer:critic'):
        if name not in modules: continue
        state = modules[name].state_dict()
        if state.keys() != saved[name].keys() or any(state[k].shape != saved[name][k].shape for k in state):
            raise RuntimeError('Incompatible checkpoint module: ' + name)
    original_load(self, load_path, policy, reset_global_steps)
    for name in ('Policy', 'Optimizer:critic'):
        if name not in modules: continue
        if any(not torch.equal(v.cpu(), saved[name][k].cpu()) for k, v in modules[name].state_dict().items()):
            raise RuntimeError('Checkpoint transfer did not preserve ' + name)
    print('ROVER_CHECKPOINT_VERIFIED: actor and critic match; 21 observations, 2 actions.', flush=True)

TorchModelSaver._load_model = checked_load
TorchModelSaver.save_checkpoint = guarded_save
TrainerController.advance = guarded_advance
if __name__ == '__main__': main()
