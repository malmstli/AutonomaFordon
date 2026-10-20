"""Prepare a compatible 11/21-input seed or export a 21-input model; preserve the source."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import torch
import onnx
from mlagents_envs.base_env import ObservationSpec, DimensionProperty, ObservationType, ActionSpec, BehaviorSpec
from mlagents.trainers.settings import NetworkSettings
from mlagents.trainers.policy.torch_policy import TorchPolicy
from mlagents.trainers.torch_entities.model_serialization import ModelSerializer
from mlagents.trainers.torch_entities.networks import SimpleActor

torch.set_num_threads(1)
def policy(size):
    obs = ObservationSpec((size,), (DimensionProperty.NONE,), ObservationType.DEFAULT, 'VectorSensor')
    return TorchPolicy(123, BehaviorSpec([obs], ActionSpec(2, ())), NetworkSettings(hidden_units=128, num_layers=2, normalize=False), SimpleActor, {})

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--export-only', action='store_true', help='Export an already-trained 21-input checkpoint; refuse to overwrite.')
    args = parser.parse_args()
    if args.export_only:
        if args.destination.suffix.lower() != '.onnx': raise ValueError('Output must end in .onnx')
        if args.destination.exists() or args.destination.with_suffix('.provenance.json').exists(): raise FileExistsError(args.destination)
        trained = torch.load(args.source, map_location='cpu')
        final = policy(21)
        final.actor.load_state_dict(trained['Policy'], strict=True)
        args.destination.parent.mkdir(parents=True, exist_ok=True)
        ModelSerializer(final).export_policy_model(str(args.destination.with_suffix('')))
        onnx.checker.check_model(onnx.load(str(args.destination)))
        args.destination.with_suffix('.provenance.json').write_text(json.dumps({
            'source': str(args.source.resolve()), 'source_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(),
            'source_step': int(next(iter(trained['global_step'].values())).item()), 'observations': 21, 'actions': 2}, indent=2))
        print('Verified 21-input, 2-action export:', args.destination)
        return
    target = args.destination / 'Rover'
    if args.destination.exists(): raise FileExistsError('Use a new seed directory; existing files are protected.')
    original = torch.load(args.source, map_location='cpu')
    seed = copy.deepcopy(original)
    key = 'network_body._body_endoder.seq_layers.0.weight'
    input_size = seed['Policy'][key].shape[1]
    assert input_size in (11, 21), input_size
    for module in ('Policy', 'Optimizer:critic'):
        old = seed[module][key]
        assert old.shape == (128, input_size)
        if input_size == 11:
            seed[module][key] = torch.cat((old, torch.zeros(128, 10)), dim=1)
    for values in seed['Optimizer:value_optimizer']['state'].values():
        for name, value in list(values.items()):
            if torch.is_tensor(value) and value.shape == (128, 11):
                values[name] = torch.cat((value, torch.zeros(128, 10)), dim=1)
    before, after = policy(input_size), policy(21)
    before.actor.load_state_dict(original['Policy'], strict=True)
    after.actor.load_state_dict(seed['Policy'], strict=True)
    torch.manual_seed(123)
    obs = torch.randn(512, input_size)
    with torch.no_grad():
        left = before.actor([obs])[-1]
        right = after.actor([torch.cat((obs, torch.zeros(512, 10)), dim=1) if input_size == 11 else obs])[-1]
    error = float((left - right).abs().max())
    assert torch.allclose(left, right, atol=1e-6, rtol=1e-6), error
    target.mkdir(parents=True)
    torch.save(seed, target / 'checkpoint.pt')
    ModelSerializer(after).export_policy_model(str(target / 'RoverFinal_Seed'))
    onnx.checker.check_model(onnx.load(str(target / 'RoverFinal_Seed.onnx')))
    provenance = {'source': str(args.source.resolve()), 'source_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(),
        'source_step': int(next(iter(original['global_step'].values())).item()), 'observations': [input_size, 21], 'actions': 2,
        'migration': 'Ten zero sensor columns added.' if input_size == 11 else 'Compatible 21-input checkpoint copied unchanged.',
        'zero_ray_action_max_error': error, 'training': 'none; initialize-from resets the new run step count'}
    (args.destination / 'provenance.json').write_text(json.dumps(provenance, indent=2))
    print(json.dumps(provenance, indent=2))

if __name__ == '__main__': main()
