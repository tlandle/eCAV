# eval.py
from argparse import ArgumentParser
import pytorch_lightning as pl
from torch_geometric.loader import DataLoader

from smart.datasets.scalable_dataset import MultiDataset
from smart.model import SMART
from smart.transforms import WaymoTargetBuilder
from smart.utils.config import load_config_act
from smart.utils.log import Logging

def make_dataset(cfg, split: str):
    """Return a MultiDataset and its DataLoader for the requested split."""
    raw_dir  = cfg.Dataset.val_raw_dir  if split == 'val' else cfg.Dataset.test_raw_dir
    proc_dir = cfg.Dataset.val_processed_dir if split == 'val' else cfg.Dataset.test_processed_dir

    dataset_cls = {"scalable": MultiDataset}[cfg.Dataset.dataset]
    ds = dataset_cls(
        root=cfg.Dataset.root,
        split=split,
        raw_dir=raw_dir,
        processed_dir=proc_dir,
        transform=WaymoTargetBuilder(
            cfg.Model.num_historical_steps,
            cfg.Model.decoder.num_future_steps
        )
    )
    dl = DataLoader(
        ds,
        batch_size=cfg.Dataset.batch_size,
        shuffle=False,
        num_workers=cfg.Dataset.num_workers,
        pin_memory=cfg.Dataset.pin_memory,
        persistent_workers=cfg.Dataset.num_workers > 0,
        collate_fn=lambda b: [x for x in b if x is not None]
    )
    return dl

if __name__ == '__main__':
    pl.seed_everything(2, workers=True)

    parser = ArgumentParser()
    parser.add_argument('--config', type=str,
                        default="configs/validation/validation_scalable.yaml")
    parser.add_argument('--split', type=str, choices=['val', 'test'],
                        default='val', help="dataset split to evaluate")
    parser.add_argument('--pretrain_ckpt', type=str, default="",
                        help="path to .ckpt with model weights")
    args = parser.parse_args()

    cfg = load_config_act(args.config)

    # ----- dataset & dataloader ------------------------------------------------
    dataloader = make_dataset(cfg, args.split)

    # ----- model ---------------------------------------------------------------
    model = SMART(cfg.Model)
    if args.pretrain_ckpt:
        logger = Logging().log(level='DEBUG')
        model.load_params_from_file(filename=args.pretrain_ckpt, logger=logger)

    # enable trajectory rollout so validation_step logs minADE/FDE
    model.inference_token = True

    # ----- trainer -------------------------------------------------------------
    trainer = pl.Trainer(
        accelerator=cfg.Trainer.accelerator,
        devices=cfg.Trainer.devices,
        strategy='ddp',
        num_sanity_val_steps=0
    )

    # Lightning will print val_loss, val_cls_acc, val_minADE, val_minFDE
    trainer.validate(model, dataloaders=dataloader)
