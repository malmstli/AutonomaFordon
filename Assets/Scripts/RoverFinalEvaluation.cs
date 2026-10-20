using System;
using System.IO;
using System.Text;
using System.Globalization;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Policies;
using Unity.InferenceEngine;

// Optional evaluation only. Training is unchanged while this toggle is off.
[DefaultExecutionOrder(-10000)]
[RequireComponent(typeof(RoverAgent))]
public class RoverFinalEvaluation : MonoBehaviour
{
    public bool enableEvaluation;
    public ModelAsset model;
    public bool allLevels = true;
    [Range(1,5)] public int selectedLevel = 3;
    [Min(1)] public int episodesPerLevel = 20;
    public int baseSeed = 830000;
    [Min(1)] public float evaluationTimeScale = 20;
    public bool quitPlayerWhenFinished;
    private RoverAgent agent;
    private BehaviorParameters behavior;
    private int level, episode, total, success, levelSuccess;
    private bool running, reversed;
    private float originalTimeScale;
    private readonly StringBuilder rows = new StringBuilder();
    private string reportPath;
    private string initialPose;

    void Awake()
    {
        if(!enableEvaluation)return;
        agent=GetComponent<RoverAgent>();behavior=GetComponent<BehaviorParameters>();
        if(model==null||!agent.obstacleObservations||behavior.BrainParameters.VectorObservationSize!=21)
        { Debug.LogError("Evaluation requires a compatible 21-input final model and scene.",this);enabled=false;return; }
        behavior.Model=model;behavior.BehaviorType=BehaviorType.InferenceOnly;
        behavior.InferenceDevice=InferenceDevice.Burst;behavior.DeterministicInference=true;
        level=allLevels?1:selectedLevel;
        agent.level=(RoverAgent.RoverLevel)level;agent.trainAcrossLevels=false;
        agent.fixedObstacleCount=6;
        agent.EvaluationSettings=true;
        agent.resetAfterSuccess=true;agent.hillDifficulty=0.5f;
    }
    void Start()
    {
        if(!enableEvaluation||!enabled)return;
        if(Academy.Instance.IsCommunicatorOn)
        { Debug.LogError("Stop the trainer before evaluation.",this);enabled=false;return; }
        originalTimeScale=Time.timeScale;Time.timeScale=evaluationTimeScale;
        running=true;agent.PauseAfterEpisode=false;
        agent.EpisodeFinished+=Finished;
        agent.EpisodeStarted+=Started;
        string folder=Path.GetFullPath("EvaluationReports/RoverFinal");Directory.CreateDirectory(folder);
        reportPath=Path.Combine(folder,DateTime.UtcNow.ToString("yyyyMMdd_HHmmss_fffffff")+"_"+model.name+".csv");
        rows.AppendLine("# deterministic=true; hills=0.5; fixed_dt="+Time.fixedDeltaTime.ToString("R",CultureInfo.InvariantCulture)+"; model="+model.name);
        rows.AppendLine("level,episode,seed,success,reason,seconds,initial_distance,closest_distance,final_distance,entered_radius,reversed,rover_x,rover_y,rover_z,yaw,goal_x,goal_y,goal_z,obstacles,blocked_direct_path");
        agent.SetNextEpisodeSeed(Seed());agent.EndEpisode();
    }
    int Seed()=>baseSeed+level*1000+episode+1;
    void Started()
    {
        var body=agent.GetComponent<Rigidbody>();var p=body.position;var g=agent.meetingPoint.position;
        initialPose=string.Join(",",p.x.ToString("F4",CultureInfo.InvariantCulture),p.y.ToString("F4",CultureInfo.InvariantCulture),
            p.z.ToString("F4",CultureInfo.InvariantCulture),body.rotation.eulerAngles.y.ToString("F3",CultureInfo.InvariantCulture),
            g.x.ToString("F4",CultureInfo.InvariantCulture),g.y.ToString("F4",CultureInfo.InvariantCulture),g.z.ToString("F4",CultureInfo.InvariantCulture),agent.obstacleField.Count,agent.InitialPathBlocked?1:0);
    }
    void FixedUpdate()
    {
        if(!running)return;
        if(!enableEvaluation){Stop();return;}
        var body=agent.GetComponent<Rigidbody>();
        reversed|=Vector3.Dot(body.linearVelocity,body.rotation*Vector3.forward)<-0.25f;
        if(!agent.enabled){Debug.LogError("Invalid evaluation spawn: aborting without skipping the seed.",this);Stop();}
    }
    void Finished(bool ok,string reason,float duration,float initial,float closest,float last)
    {
        if(!running)return;
        rows.AppendLine(string.Join(",",level,episode+1,Seed(),ok?1:0,reason,
            duration.ToString("F3",CultureInfo.InvariantCulture),initial.ToString("F4",CultureInfo.InvariantCulture),
            closest.ToString("F4",CultureInfo.InvariantCulture),last.ToString("F4",CultureInfo.InvariantCulture),
            closest<agent.targetDistance?1:0,reversed?1:0)+","+initialPose);
        total++;if(ok){success++;levelSuccess++;}episode++;reversed=false;
        File.WriteAllText(reportPath,rows.ToString());
        if(episode==episodesPerLevel)
        {
            Debug.Log($"ROVER_FINAL_LEVEL: model={model.name}; level={level}; successful_stops={levelSuccess}/{episode}",this);
            if(!allLevels||level==5)
            {
                agent.PauseAfterEpisode=true;
                Debug.Log($"ROVER_FINAL_COMPLETE: {success}/{total} successful stops; report={reportPath}",this);
                Stop();
                if(quitPlayerWhenFinished&&!Application.isEditor)Application.Quit(0);
                return;
            }
            level++;episode=levelSuccess=0;agent.level=(RoverAgent.RoverLevel)level;
        }
        agent.SetNextEpisodeSeed(Seed());
    }
    void Stop()
    {
        if(!running)return;
        running=false;Time.timeScale=originalTimeScale;
        agent.EpisodeFinished-=Finished;agent.carControl.ResetControls();
        agent.EpisodeStarted-=Started;
        if(File.Exists(reportPath))File.WriteAllText(reportPath,rows.ToString());
    }
    void OnDisable(){Stop();}
    void OnDestroy(){Stop();}
}
