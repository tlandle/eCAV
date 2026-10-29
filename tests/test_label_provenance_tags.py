"""The ground-truth brake labeler tags a track's provenance against the consuming ego, so another
CAV's track is 'nonego' for every ego but its own. Runs standalone."""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecav.core.application.edge.edge_manager.edge_manager_base import provenance_tags_for  # noqa: E402


def main():
    # track of CAV 159, consumed by CAV 109: a real other vehicle, not a self-ghost
    hist = [('ego', 159, 0.3), ('ego', 159, 0.4), ('ego', 159, 0.2)]
    assert provenance_tags_for(hist, 109) == ['nonego'] * 3
    # the same track consumed by CAV 159 itself: its own echo
    assert provenance_tags_for(hist, 159) == ['ego'] * 3
    # one managed ego: the stored tag and the id agree
    assert provenance_tags_for([('ego', 7, 0.1), ('nonego', 900, 0.5)], 7) == ['ego', 'nonego']
    # a merge between the consumer and a non-managed actor stays a merge
    assert provenance_tags_for([('ego', 7, 0.1), ('nonego', 900, 0.5)], 7).count('ego') == 1
    # legacy entries without an id keep the stored tag
    assert provenance_tags_for([('ego',), ('nonego',)], 7) == ['ego', 'nonego']
    print("ok")


if __name__ == "__main__":
    main()
