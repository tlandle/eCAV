# DO NOT USE — provenance unknown (quarantined 2026-08-16)

Facts from this folder alone:
- config.yaml + result.txt dated Apr 19; the three net_epoch{5,7,16}.pth
  are dated Jun 16 — the checkpoints were added two months after the
  config, origin unrecorded.
- Despite the folder name, these were NEVER trained on the Azure NDm
  (Tyler; Azure was out in the April window). Training machine unknown.
- The config claims MultiV2XIntermediateFusionDataset, but a copied config
  does not establish what the June .pth files were trained on. Dataset
  unconfirmed (possibly OPV2V-era).
- config lacks random_world_translation, so even if the metrics were real
  (result.txt AP@0.5 0.627, offline probes ep16 0.820 occluded recall)
  they carry the scene-overfit risk proven for such recipes.

Do not use these checkpoints for live runs, finetune bases, or paper
numbers. If they are ever needed, provenance must be re-established from
training logs first.
