using BehaviorDesigner.Runtime;
using BehaviorDesigner.Runtime.Tasks;
using UnityEngine;
using UnityEngine.AI;

public class CanSeekTarget : Conditional
{

    public SharedGameObject NPC;
    private NPC npc;
    private NavMeshAgent agent;
    private LYHSensor.TriggerSensor triggerSensor;

    public SharedGameObject target;
    public override void OnStart()
    {
        base.OnStart();
        if (triggerSensor == null && NPC.Value != null)
        {
            npc = NPC.Value.GetComponent<NPC>();
            agent = NPC.Value.GetComponent<NavMeshAgent>();
            triggerSensor = npc.eyeSensor;
        }

    }
    public override TaskStatus OnUpdate()
    {
        target.Value = triggerSensor.GetNearestToPoint(this.transform.position);
        if (target.Value)
        {
            return TaskStatus.Success;
        }
        return TaskStatus.Failure;
    }
}
