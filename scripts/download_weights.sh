#!/usr/bin/env bash
# Download GroundingDINO + MobileSAM checkpoints and the BERT text encoder GroundingDINO needs.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CK="$ROOT/perception/ckpts"
mkdir -p "$CK/gdino" "$CK/mobilesam"
dl() { [ -s "$2" ] && { echo "have $2"; return; }; wget -c -q --show-progress "$1" -O "$2"; }
dl https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth "$CK/gdino/gdino.pth"
dl https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt "$CK/mobilesam/vit_t.pth"
# shellcheck disable=SC1091
source "$ROOT/.venv/bin/activate"
python - <<'PY'
from transformers import BertModel, BertTokenizer
BertTokenizer.from_pretrained("bert-base-uncased"); BertModel.from_pretrained("bert-base-uncased")
print("bert-base-uncased cached")
PY
echo "[ok] weights ready in $CK"
