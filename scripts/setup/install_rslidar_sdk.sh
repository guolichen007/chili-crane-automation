#!/usr/bin/env sh
set -eu
# Explicit vendor workspace only. Does not install system packages or run sensors.
if [ "$#" -ne 2 ] || [ "$1" != "--workspace" ]; then
  printf '%s\n' 'usage: install_rslidar_sdk.sh --workspace <isolated-vendor-workspace>' >&2
  exit 2
fi
repo=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
workspace=$2
case "$workspace" in ''|/|.) printf '%s\n' 'explicit isolated workspace required' >&2; exit 2 ;; esac
mkdir -p "$workspace/src"
workspace=$(CDPATH= cd -- "$workspace" && pwd)
lock="$repo/third_party/rslidar_sdk.lock.yaml"
pin() { python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))[sys.argv[2]][sys.argv[3]])' "$lock" "$1" "$2"; }
checkout() {
  name=$1
  url=$(pin "$name" url)
  sha=$(pin "$name" commit)
  dest="$workspace/src/$name"
  if [ ! -d "$dest" ]; then
    git clone --no-checkout "$url" "$dest"
    git -C "$dest" checkout --detach "$sha"
  fi
  [ "$(git -C "$dest" remote get-url origin)" = "$url" ]
  [ "$(git -C "$dest" rev-parse HEAD)" = "$sha" ]
}
checkout sdk
checkout rslidar_msg
sdk="$workspace/src/sdk"
git -C "$sdk" submodule update --init --recursive
[ "$(git -C "$sdk/src/rs_driver" rev-parse HEAD)" = "$(pin rs_driver commit)" ]
patch="$repo/third_party/rslidar_xyzirt.patch"
if git -C "$sdk" apply --reverse --check "$patch" 2>/dev/null; then
  : # Already patched; never reset a checkout.
else
  [ -z "$(git -C "$sdk" status --porcelain --untracked-files=no)" ]
  git -C "$sdk" apply --check "$patch"
  git -C "$sdk" apply "$patch"
fi
python3 "$repo/tools/check_vendor_checkout.py" "$workspace"
printf '%s\n' 'SDK pinned; XYZIRT patch applied; NO_DO_WRITE; driver NOT started'
git -C "$sdk" rev-parse HEAD
git -C "$sdk/src/rs_driver" rev-parse HEAD
git -C "$workspace/src/rslidar_msg" rev-parse HEAD
if command -v dpkg-query >/dev/null 2>&1; then
  dpkg-query -W gcc g++ libyaml-cpp-dev libpcap-dev > "$workspace/vendor-build-environment.txt"
fi
sha256sum "$patch" "$lock" > "$workspace/vendor-provenance.sha256"
