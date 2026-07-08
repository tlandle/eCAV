# Plan of attack: Evaluation (§5, evaluation4.tex)

Goal: tighten flow, add section-flow + per-result intent tags (bulleted), and resolve every visible comment and figure inconsistency. Structure is sound (claim -> figure -> interpretation -> takeaway); this is cleanup + consistency, not a rewrite. Keep wording simple; reuse existing prose; keep \ad/\KR markers and annotate with \tl{Fixed:}.

## A. Flow assessment

Current order:
- 5.1 Methodology: Platform, Metrics, Evaluated systems, Evaluation Groups, Latency/Duse models, Scenarios, Participant Scaling.
- 5.2 Results: roadmap + Envelope, Situation, Landing(+margin), Failure channel, Source independence, Ablation.
- 5.3 Overhead.

Problems:
1. Methodology order is jumbled. Metrics appears before Scenarios and before the systems are even introduced; "Evaluation Groups" overlaps the "two measurement modes" in the Latency subsection; the situation-study sentence is buried at the top of the Latency subsection.
2. Naming inconsistency: "Evaluation Groups" vs "landing study/envelope sweep" (Alex L49: rename groups -> studies). "situation study" used before it is defined (L48).
3. Opening sentence (L4) is methodology-flavored prose that Alex (L5) wants moved/cut.
4. Takeaway numbering is broken: 1, 1b, 2, 3, 4, 5, 6 (Alex L109: why 1b not 2).
5. Roadmap (L78) promises four questions but there are six results + overhead; mismatch.
6. Two high-level points are missing (Alex L90): collaborative vs local-only value, and sharing+contract beating the single-source Oracle.

Proposed methodology order: Platform -> Scenarios -> Evaluated systems + contract dimension -> Metrics (Duse, S_op) -> Latency/Duse models (+ the two measurement modes, renamed studies) -> Participant scaling. Fold "Evaluation Groups" into the measurement-modes paragraph (envelope sweep / landing study / object-sharing ablation), so "groups" and "modes" stop competing.

Proposed results order (unchanged, renumber takeaways 1-7):
1 Envelope (one physics envelope; contract removes the early logic failure)
2 Situation budget (tau_max(u); 100 ms not universal)
3 Landing + safety margin m(a,u,N)
4 Failure channel (physics vs logic)
5 Source independence (MAC contention lands on same envelope)
6 Object-sharing ablation (alignment != identity; SBA; robustness)
7 Overhead (cheap)

## B. Section flow + intent tags (the bulleted deliverable)

Add at the §5 head a \secflow listing what the evaluation shows, as bullets:
- one situation-specific physics envelope; the contract removes the early logic failure;
- the useful-age budget is maneuver-dependent (100 ms is not a universal target);
- where each architecture lands as N grows, and its safety margin;
- failures split into physics vs logic; the contract removes the logic class;
- safety depends on age at use, not the delay source;
- alignment fixes when, not whose; SBA enforces identity cheaply.

Add one \ptag per results subsubsection stating exactly what that result demonstrates (single sentence each, mirroring the takeaway). Methodology subsubsections get light \ptag tags too (what each sets up).

## C. Comment resolution (visible \ad; keep marker + \tl{Fixed:})

- L5 opening placement -> move L4 into Results intro or cut; start §5 with Methodology.
- L42 evaluation-group opening -> already has \tl{Fixed}; confirm the new systems subsection satisfies it, then comment the \ad per approval.
- L48 "what is a situation study?" -> define it where first used (the situation budget subsection), or rename to "situation sweep" and gloss.
- L48 "proper opening for latency modeling?" -> move the situation-rerun sentence out of the Latency subsection opener; start it with the two measurement modes.
- L49 groups vs studies -> rename "Evaluation Groups" to measurement modes/studies and unify with the modes paragraph.
- L57 scenario names text vs captions -> unify names (LTAP/OD, SCP) between prose and float-scenarios caption.
- L88 "where is the 100-200 ms collapse? show on figure" -> confirm the no-contract curves are on fig:arch-safety-scale and annotate the collapse band, or cite the panel.
- L90 two high-level points -> add a sentence in the envelope result: collaboration beats local-only, and sharing + contract can match/beat the single-source Oracle.
- L109 "why 1b not 2" -> renumber takeaways 1-7.
- L117 "local age grows with N? local above VRF?" -> FIGURE/data check (see D); fix the figure labels or explain in text.
- L122 "spell out compact object sharing" -> name the systems (CIP, I2V object sharing, V2X2V prediction).
- L149 unparseable "this/not a weak detector" -> rewrite the sentence plainly.
- L150 "Anchoring -> the contract?" -> already changed to "Contract introduction"; tighten.
- L176 "use the contract abstractly" unclear -> reword: earlier results treat the contract as an idealized on/off; here we use the actual SBA implementation.
- L181 "ghost-free rate visible? N for Fig 13a?" -> FIGURE check; state the N and point to the correct panel, or add the ghost-free series.
- L191 "what invariant? better verb than bought" -> reword ("the invariant is cheap; it does not require unrealistic edge compute").

Commented \tl (L13, L50, L60, L64, L70, L160): already commented; skip per the no-already-commented rule, except note L160 (uplink vs sidelink) and L60 (red-light scenario) as open questions for you.

## D. Figure inconsistencies to verify and fix

1. fig:arch-landing (arch_landing.pdf): does it plot local-only, and is it above VRF? Resolve the "local floor vs VRF" confusion (Alex L117). Likely the curve is the "local compute floor (Oracle compute)", not "local-only"; relabel or clarify, and explain any N-dependence (or remove it if local should be flat).
2. float-scenarios caption vs prose scenario names (L57).
3. fig:sba-ablation: confirm N for panel (a) and whether the ghost-free rate is shown; align the text (L181).
4. fig:arch-safety-scale: ensure no-contract collapse (~100-200 ms) is visible/annotated (L88).
5. Cross-check every number in prose against the figure/table it cites (landings 174/198, 181/209, 189/222; budget 220-450; safe boundary ~280 ms; PRR; overhead 9.9 KB/s, <0.2 ms, ~190 ms RT).

## E. Sequencing

Phase 1 - Methodology: reorder subsubsections; move/cut the opening line; rename groups->studies and merge with modes; resolve L5, L42, L48, L49, L57; light \ptag tags.
Phase 2 - Results framing: add \secflow (bulleted) and per-result \ptag; renumber takeaways 1-7; align the roadmap; add the two high-level points (L90).
Phase 3 - Per-result text fixes: L88, L122, L149, L150, L176, L191 (reword, reuse prose).
Phase 4 - Figure consistency: the D items (landing local/VRF, scenario names, sba N/ghost-free, safety-scale annotation, number cross-check). Regenerate figures only where a label is wrong.
Phase 5 - Compile each phase, annotate \ad with \tl{Fixed:}, push.

Each phase = one commit, compiled. Figures are produced from the measured full-matrix sweep outputs (test_runner runs per `full_envelope_matrix.md`, real C-V2X RTT trace driving the latency model); only relabel where the prose/figure disagree.
