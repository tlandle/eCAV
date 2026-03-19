
#!/usr/bin/env python3
"""
merge_latency_zips.py  -  Combine latency traces from multiple ZIP archives.

Usage:
  python merge_latency_zips.py intersection_3.zip intersection_4.zip intersection_5.zip
  # -> creates merged_latency.csv in the current directory

Each row is tagged with the intersection ID extracted from the ZIP filename
(e.g., intersection_3.zip -> intersection=3).
"""

import sys, re, csv, zipfile
from pathlib import Path

# -------- helper to process one archive --------------------------------------
_INT_RE = re.compile(r"intersection_(\d+)")

def rows_from_zip(zip_path: Path):
    """Yield dictionaries for every latency row inside one zip archive."""
    # Extract intersection ID from filename
    m_int = _INT_RE.search(zip_path.stem)
    intersection = m_int.group(1) if m_int else "unknown"

    pattern = re.compile(r".*/rx_(.+?)_tx_(.+?)_perception\.csv$")
    with zipfile.ZipFile(zip_path) as z:
        for fname in z.namelist():
            m = pattern.fullmatch(fname)
            if not m:
                continue  # skip unrelated files
            receiver, sender = m.group(1), m.group(2)
            with z.open(fname) as f:
                reader = csv.DictReader(
                    (line.decode("utf-8", errors="ignore") for line in f)
                )
                for row in reader:
                    lat = row.get("latency (ms)", "").strip()
                    ts  = row.get("rx_timestamp (us)", "").strip()
                    if lat and ts:          # keep only valid latency entries
                        yield {
                            "intersection"    : intersection,
                            "sender"          : sender,
                            "receiver"        : receiver,
                            "rx_timestamp_us" : ts,
                            "latency_ms"      : lat,
                        }

# -------- main ---------------------------------------------------------------
def main(zip_files):
    out_file = Path("merged_latency.csv")
    fieldnames = ["intersection", "sender", "receiver", "rx_timestamp_us", "latency_ms"]

    with out_file.open("w", newline="") as fout:
        writer = csv.DictWriter(fout, fieldnames=fieldnames)
        writer.writeheader()

        row_count = 0
        for z in zip_files:
            for rec in rows_from_zip(Path(z)):
                writer.writerow(rec)
                row_count += 1

    print(f"Wrote {row_count:,} rows to {out_file.resolve()}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("Usage: python merge_latency_zips.py archive1.zip [archive2.zip ...]")
    main(sys.argv[1:])
