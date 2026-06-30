# Notes: Tyler - Modular Edge Stack

**Date:** 2026.05.31
**Source:** weekly meeting

---

## Modular Edge Stack

The edge stack is designed to be modular with three full independent components:
- fusion: world, late, or even possibly early (point cloud)
- tracker: ab3dmot,  mamba, etc
- predictor: SMART, linear, etc

This means any scenario should be runnable with all any/all permutations.

There are other modular components such as the Network Model that fall into this same pattern of removable/addable blocks that each represent an area of research focus.

---

## Edge-To-Edge Hand-Off

We need to build a new module for the edge - `handoff_manager`. This will be responsible for transferring state between edges as a vehicle crosses a locale boundary.

In the simplest implementation, this is purely naive and optimal - state is transferred instantly and perfectly (something like shared memory). We could imagine in this case even something like a broadcast where every time the owning edge updated state it merely broadcasts that state to all other edges (nearby or otherwise) so those edges are fully warm when the vehicle enters their locale. 

Also important is the idea of locale-to-locale transfer for an edge that manages multiple locales, though it's not clear how practically different this is from edge-to-edge transfer.

The simple test experiment we want to design builds on top of our `-eo` work of distributing just the edge. For our simplest scenario, we should imagine an ego vehicle crossing a locale boundary (where locales will overlap for some `N` number of meters) and there are two edges and the vehicle simply travels between them and we have an extensible `handoff_manager` that manages the state transition (and which we must be able to build upon).

Did an initial draft around this here - [multi_edge_locale_handoff](obsidian://open?vault=eCAV&file=docs%2Fagent_plans%2Fmulti_edge_locale_handoff).

---

## Branch Cleanup

Need to pull in the `develop` branch. It has the same `ab3dmot` fix and also a lot more. But given that the `ab3dmot` change is the same - just trimming the list bounds - it's not clear we need to do this now. But we should aim to get on the same working branch _soon_.