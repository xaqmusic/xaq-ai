"""Run logs may be plain or gzip-compressed (2026-10-22: 175 GB of per-tick logs filled the disk, so the harness now
compresses each log once it has been scored).  open_log() reads either; log_glob(pat) matches `pat` and `pat.gz`."""
import glob, gzip


def open_log(path):
    path = str(path)
    if path.endswith(".gz"):
        return gzip.open(path, "rt", errors="replace")
    return open(path, errors="replace")


def log_glob(pat):
    return sorted(set(glob.glob(pat)) | set(glob.glob(pat + ".gz")))
