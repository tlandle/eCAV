# Paper 3 draft section: closed-loop results (flow scenario)

Data: eval_flow_v2.csv (5 arms x 4 reps, occlusion-aware controlled
perception, steady 8-vehicle oncoming stream at 5 s headway, blind
overtake, scheduled end). Figure: paper_flow_headline.{pdf,png}.

## Results prose (draft)

We evaluate the closed loop under steady oncoming traffic. The ego must
overtake a stopped truck through the opposing lane. Oncoming vehicles
arrive every 5 seconds from an upstream locale, and the run ends at a
fixed deadline. A run succeeds when the ego completes the overtake within
the deadline without a collision. This design measures safety and
liveness together: an arm can fail by colliding, by never finding a gap,
or both.

Predictive latent migration completes every run with zero collisions
(4/4 runs, 21 to 29 s to completion). Every baseline fails at least one
way. The snapshot arm completes no runs and averages 5.5 collision
episodes: a one-frame state carries no velocity, so the destination
predicts every migrated vehicle as stationary and the ego commits into
closing traffic. EdgeWarp completes half its runs and averages 5.3
episodes; its blocking transfer at the crossing leaves the destination
blind to each arriving vehicle for the transfer plus track re-maturation,
and in continuous traffic some vehicle is always inside that window.
The reactive arm, which carries our full latent but fires only at the
crossing, still collides 3 to 4 times per run: timing alone breaks the
protocol even when the content is right. Cold start completes 3 of 4
runs by waiting out the traffic, and its fourth run records 10 collision
episodes, the worst single run in the table.

The gap the flow closes: with a single oncoming vehicle, a blind arm can
wait until the road happens to clear and look safe. Under a steady flow
there is no such window to luck into. Only the arm that receives each
vehicle's motion history before the vehicle arrives can both wait
correctly and go decisively.

## Numbers (for tables)

| arm | completed | mean episodes | completion time |
|---|---|---|---|
| ours (predictive latent) | 4/4 | 0.0 | 21-29 s |
| reactive (timing ablation) | 3/4 | 3.75 | 14-51 s |
| snapshot (content ablation) | 0/4 | 5.5 | never |
| EdgeWarp | 2/4 | 5.25 | 21-50 s |
| cold | 3/4 | 3.5 | 23-52 s |

Caveats for the text: controlled perception (GT detections with sensing
range and line-of-sight occlusion), 4 reps per arm (more seeds queued),
single scenario geometry (Town01 blind overtake; the merge geometry and
live-perception runs follow the WF retrain).
