# Plan: Address advisor comments (SenSys safety-envelope paper)

Source: comments pulled from Overleaf into `safety_envelope_sensys` (master `df56808`).
Authors: `\ad{}` = professor (highest priority, preserve), `\KR{}` = senior reviewer, `\tl{}` = Tyler self-notes.
Rule: resolve substance first, then comment out the marker (`%\ad{...}`), never delete `\ad{}`/`\KR{}`. Don't orphan Overleaf sidecar comments by rewriting anchor text wholesale.

Only ACTIVE files are in scope: abstract, introduction, related_work, system_architecture, evaluation4, discussion, conclusion, main.tex, and the floats those input. Comments in evaluation.tex / evaluation3.tex / inactive floats are ignored (not in the build).

---

## Tier 1 — Intro + Abstract (make-or-break; KR flagged explicitly)

KR intro:4 is the governing comment: intro reads as stream-of-consciousness, terse and jumpy; add a one-line intent comment atop each paragraph then flesh out; same for abstract.

- intro:4 (KR) restructure intro: per-paragraph intent line + expand. Fixes the "no discernible structure" complaint.
- intro:21 (ad) reorder: setup (multi-AV, RSU) -> prior coop perception -> RQs, and PUNT all evaluation-framework detail to AFTER the RQs. Remove the platform/concepts/platform zig-zag.
- intro:30 (ad) "TMI for the intro" - cut the over-detailed sentence flagged.
- intro:3,5 (KR) occlusion is not the only problem: add limited line of sight and BLOS (cite ~300 m LoS).
- intro:13 (ad) add citation(s) for the flagged claim.
- abstract:3 (ad) first paragraph too esoteric for a non-AV SEC reader; raise one level of abstraction (role of edge in AV before ghosts).
- abstract:30 (ad) "back to esoteric language" - same fix, plain-language pass.
- abstract:15,16 (ad) two unclear sentences ("what does this mean", "what curves?") - rewrite or cut.

Output: one focused rewrite of abstract + intro. Highest leverage. Do first.

## Tier 2 — Architecture §3 (repetition, dangling sentences, undefined terms)

- sysarch:24 (ad) dangling sentence + the following para "comes out of nowhere"; clarify how the contract relates to instrumenting the pipeline (the "We instrument the pipeline..." para). Likely move that para to §5.1.
- sysarch:29 (ad) eCloudSim is deanonymized: decide naming (use official name, not CAValier; don't imply we built it); sim-env info is repeated in §5.1.1 - remove from §3 (why is the simulator in the architecture-design section at all).
- sysarch:32 (ad) info repeated in §5.1.3 - dedup.
- sysarch:67 (ad) "broken" - fix the broken sentence/ref at line 67.
- sysarch:8 (tl) needs citation; Fig 1 stack is "definitively V2X2V" - relabel/fix so the stack reads as the general pipeline, not V2X2V-only.
- sysarch:23 (tl) drop or justify the forward pointer to results inside §3.
- sysarch:74 (KR) self-ghosting paragraph too terse - expand so the reviewer sees exactly when self-ghosting occurs.
- sysarch:97 (KR) define or remove "bounded anchoring epoch".
- sysarch:99 (KR) define "geometric gating"; fix "reject what?" dangling clause.
- sysarch:107 (KR) define "decouple".
- sysarch:103 (tl) add a sentence on beacon-id-manager security posture.
- sysarch:106 (KR) cut or justify the para whose relevance to beaconing is unclear.

## Tier 3 — Evaluation §5 (structure + specific data questions)

- eval4:25 (ad) add a subsection BEFORE evaluation groups that defines every architecture evaluated and the on/off contract dimension; then say what an "evaluation group" groups.
- eval4:5 (ad) reconsider the section opening; consider moving it under Metrics and starting with Methodology.
- eval4:30 (ad) define "situation study"; check the latency-modeling opening sentence.
- eval4:31 (ad) unify names: "landing study"/"envelope sweep" vs earlier "landing group"/"envelope group" - pick one.
- eval4:39 (ad) scenario names must match between text and captions (LTAP/OD, SCP).
- eval4:72 (ad) add the two missing high-level points: (i) collaborative vs local-only value, (ii) sharing + contract can beat a single-source oracle.
- eval4:91 (ad) fix Takeaway numbering ("why 1b not 2?").
- eval4:99 (ad) explain why local-only age-at-use grows with N, and why the local floor sits above VRF (or fix the figure if wrong).
- eval4:70 (ad) point to where the claim is visible; annotate the figure.
- eval4:104 (ad) spell out which systems the statement refers to.
- eval4:13 (ad) the commented-out statement: confirm it stays out.
- eval4:138 (tl) clarify uplink vs sidelink for contention; state what VRF/CIP use; likely "uplink".

## Tier 4 — Figures and tables

- float-arch-safety-scale:4 (ad) legend hides local-only: make two-column, move over plot; rename "Oracle" -> "Single-source oracle"; add a vertical dashed line at the physics envelope.
- float-arch-landing:6 (tl) graph is busy: explain prediction vs intermediate fusion, why prediction ~ I2V objects, why Oracle is on it, whether it matches CIP's real results; add an intermediate-fusion gloss.
- fig-stack-architecture:97 (tl) define the color legend (green/red/orange), make the pipeline more descriptive, fix panel (b) placement.
- sysarch:65 (KR) Figure 2 not self-explanatory - expand the figure text/caption.
- float-perception-overview:6 (ad) caption one word too long; ALSO the Fig 1 image still has "Tracker (SORT)" baked in (source diagram edit needed: SORT -> AB3DMOT).
- tbl-loss-sensitivity:4 (ad) consistent terminology; clarify whether "hybrid" = the trace-based model from the methodology.
- main.tex:112 (ad) title: replace "under tail latency" with "under network-induced latency unpredictability" (or similar); tail latency is just a percentile.
- tbl-arch-payload:11 (tl) table width - DONE (`df56808`).

## Tier 5 — Stale-flag sweep (Tyler self-notes; verify vs current text, fix or clear)

- intro:25 (tl) define CAVs earlier - likely already done; verify.
- intro:28 (tl) "up to 32 vehicles" - verify wording reflects 32.
- intro:31 (tl) "This is not true" - find and correct the false statement.
- intro:35 (tl) "out of date" - update.
- intro:42 (tl) "still good, maybe updates" - light pass.
- intro:66 (tl) intro restates itself - dedup (folds into Tier 1).
- intro:68 (tl) say "ns-3 cosimulated + trace".
- intro:79 (tl) roadmap sentence needs updating.
- eval4:32 (tl) "replaced with ns-3 cosim?" - confirm and update.
- eval4:42 (tl) do we test a run-red-light scenario? - confirm scenario list.
- eval4:46 (tl) specify the vehicle stack earlier.
- eval4:52 (tl) "also goes up to 32" - verify.

## Tier 6 — No action

- related_work:25 (KR) "kudos, comprehensive and well-written" - leave.
- sysarch:66 (KR) "spatial gate" - marked Fixed; confirm then comment out.

---

## Sequencing

1. Tier 1 (abstract + intro rewrite) - one pass, highest impact.
2. Tier 5 stale sweep folded in while touching intro/eval.
3. Tier 2 (§3 dedup + term definitions).
4. Tier 3 (§5 structure: architecture-definitions subsection, naming, missing points, data questions).
5. Tier 4 (figures/legends; the SORT image and stack-figure colors may need the diagram sources).

Each tier = one commit, compiled, then markers commented out. No push without Tyler's OK (advisor comments live on the branch).
