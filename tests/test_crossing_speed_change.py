"""The crossing car's speed change fires once, when the car is within the trigger distance of the
conflict point, by setting the follower's local planner speed (km/h). Runs standalone with stubs."""
import os
import sys
import types
from types import SimpleNamespace
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# stub py_trees enough for the behaviour's base class and status
pt = types.ModuleType("py_trees"); pt.behaviour = types.ModuleType("behaviour"); pt.common = types.ModuleType("common")
class _B:
    def __init__(self, name=""): self.name = name
pt.behaviour.Behaviour = _B
pt.common.Status = SimpleNamespace(SUCCESS="SUCCESS", RUNNING="RUNNING", FAILURE="FAILURE")
sys.modules["py_trees"] = pt; sys.modules["py_trees.behaviour"] = pt.behaviour; sys.modules["py_trees.common"] = pt.common
src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "ecav", "scenario_testing", "scenarios", "scenario_3.py")).read()
start = src.index("class CrossingSpeedChange("); end = src.index("class Scenario_3(")
ns = {"py_trees": pt}; exec(src[start:end], ns)
CrossingSpeedChange = ns["CrossingSpeedChange"]


def main():
    calls = []
    planner = SimpleNamespace(set_speed=lambda kmh: calls.append(kmh))
    class Actor:
        def __init__(self): self.loc = SimpleNamespace(x=-30.0, y=127.9)
        def get_location(self): return self.loc
    actor = Actor()
    follower = SimpleNamespace(_local_planner_dict={actor: planner})
    conflict = SimpleNamespace(x=-84.8, y=127.9)
    b = CrossingSpeedChange(actor, follower, conflict, trigger_dist=40.0, new_speed_mps=16.4)
    assert b.update() == "RUNNING" and calls == []          # 54.8 m away: nothing yet
    actor.loc = SimpleNamespace(x=-46.0, y=127.9)   # 38.8 m away
    assert b.update() == "SUCCESS" and calls == [16.4 * 3.6]
    assert b.update() == "SUCCESS" and len(calls) == 1      # fires once
    # no planner yet: waits
    follower2 = SimpleNamespace(_local_planner_dict={})
    b2 = CrossingSpeedChange(actor, follower2, conflict, 40.0, 10.4)
    assert b2.update() == "RUNNING"
    print("ok")


if __name__ == "__main__":
    main()
