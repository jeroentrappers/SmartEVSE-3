#!/usr/bin/env bash
# update_mongoose.sh — verify or upgrade the vendored Cesanta Mongoose
# library (src/mongoose.c + src/mongoose.h).
#
# Mongoose is shipped by upstream as an amalgamated single-file library;
# cesanta's own distribution model is "drop these two files into your
# project." We do exactly that, and pin the tag + commit + sha256 of
# each file in src/MONGOOSE_VERSION. This script is the only supported
# way to change that pin — upgrades then become a single reviewable
# commit: the two mongoose files + the version file all move together.
#
# Usage:
#   ./update_mongoose.sh                         # verify current pin
#   ./update_mongoose.sh --tag 7.18              # upgrade to a new tag
#   ./update_mongoose.sh --tag 7.18 --dry-run    # fetch + diff, no writes
#
# Exit status:
#   0  verify passed, or upgrade completed cleanly
#   1  tampered vendored files, or upgrade failed a checksum gate
#   2  usage / environment error
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
version_file="$here/src/MONGOOSE_VERSION"
vendored_c="$here/src/mongoose.c"
vendored_h="$here/src/mongoose.h"

read_pin() {
    # Extract `key = value` (ignoring surrounding spaces) from the version file.
    local key="$1"
    awk -v k="$key" '$1 == k { sub(/^[^=]*=[[:space:]]*/, ""); print; exit }' \
        "$version_file"
}

sha256_of() {
    # macOS ships shasum (perl). Most Linux distros ship sha256sum. Use
    # whichever is available, emit just the hex digest.
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | awk '{print $1}'
    else
        shasum -a 256 "$1" | awk '{print $1}'
    fi
}

fail() { printf 'error: %s\n' "$*" >&2; exit 1; }

[ -f "$version_file" ] || fail "missing pin file: $version_file"
[ -f "$vendored_c" ]   || fail "missing vendored mongoose.c"
[ -f "$vendored_h" ]   || fail "missing vendored mongoose.h"

pinned_tag=$(read_pin tag)
pinned_sha_c=$(read_pin sha256_mongoose_c)
pinned_sha_h=$(read_pin sha256_mongoose_h)

mode="verify"
target_tag="$pinned_tag"
dry_run=0
while [ $# -gt 0 ]; do
    case "$1" in
        --tag) mode="upgrade"; target_tag="${2:-}"; shift 2 ;;
        --dry-run) dry_run=1; shift ;;
        -h|--help)
            sed -n '1,32p' "$0" | sed -n '/^# /{s/^# \{0,1\}//;p;}'
            exit 0 ;;
        *) fail "unknown argument: $1" ;;
    esac
done

[ -n "${target_tag:-}" ] || fail "--tag requires a value (e.g. 7.18)"

verify() {
    local actual_c actual_h
    actual_c=$(sha256_of "$vendored_c")
    actual_h=$(sha256_of "$vendored_h")
    local ok=1
    if [ "$actual_c" != "$pinned_sha_c" ]; then
        printf 'MISMATCH mongoose.c\n  pinned: %s\n  actual: %s\n' \
            "$pinned_sha_c" "$actual_c" >&2
        ok=0
    fi
    if [ "$actual_h" != "$pinned_sha_h" ]; then
        printf 'MISMATCH mongoose.h\n  pinned: %s\n  actual: %s\n' \
            "$pinned_sha_h" "$actual_h" >&2
        ok=0
    fi
    if [ "$ok" = 1 ]; then
        printf 'verified: mongoose %s (commit %s)\n' \
            "$pinned_tag" "$(read_pin commit)"
        return 0
    fi
    return 1
}

if [ "$mode" = "verify" ]; then
    verify
    exit $?
fi

# --- upgrade path --------------------------------------------------------
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

base_url="https://raw.githubusercontent.com/cesanta/mongoose/$target_tag"
printf 'fetching mongoose %s\n' "$target_tag"
curl -fsSL -o "$tmp/mongoose.c" "$base_url/mongoose.c" \
    || fail "failed to download $base_url/mongoose.c"
curl -fsSL -o "$tmp/mongoose.h" "$base_url/mongoose.h" \
    || fail "failed to download $base_url/mongoose.h"

# Resolve the tag to a commit SHA for the pin file. Falls back to
# "unknown" if the GitHub API isn't reachable — the sha256 pins still
# guard integrity.
commit_sha="unknown"
if command -v curl >/dev/null 2>&1; then
    commit_sha=$(curl -fsSL \
        "https://api.github.com/repos/cesanta/mongoose/git/refs/tags/$target_tag" \
        2>/dev/null \
        | awk -F'"' '/"sha":/ { print $4; exit }' \
        || printf 'unknown')
    [ -n "$commit_sha" ] || commit_sha="unknown"
fi

new_sha_c=$(sha256_of "$tmp/mongoose.c")
new_sha_h=$(sha256_of "$tmp/mongoose.h")

# Sanity check the advertised version against what's in the downloaded
# header. Tag "7.18" in the URL should correspond to
# `#define MG_VERSION "7.18"` inside mongoose.h.
advertised=$(awk -F'"' '/^#define MG_VERSION / { print $2; exit }' \
    "$tmp/mongoose.h" || true)
if [ -n "$advertised" ] && [ "$advertised" != "$target_tag" ]; then
    printf 'warning: requested tag %s but mongoose.h reports MG_VERSION %s\n' \
        "$target_tag" "$advertised" >&2
fi

printf '\n--- upgrade summary ---\n'
printf '  tag     : %s → %s\n' "$pinned_tag" "$target_tag"
printf '  commit  : %s\n' "$commit_sha"
printf '  .c sha  : %s → %s (%s bytes)\n' \
    "$pinned_sha_c" "$new_sha_c" "$(wc -c <"$tmp/mongoose.c" | tr -d ' ')"
printf '  .h sha  : %s → %s (%s bytes)\n' \
    "$pinned_sha_h" "$new_sha_h" "$(wc -c <"$tmp/mongoose.h" | tr -d ' ')"
printf '\n'

if [ "$dry_run" = 1 ]; then
    printf 'dry-run: no files written\n'
    exit 0
fi

cp "$tmp/mongoose.c" "$vendored_c"
cp "$tmp/mongoose.h" "$vendored_h"

today=$(date -u +%Y-%m-%d)
cat >"$version_file" <<EOF
# Pinned version of the vendored Cesanta Mongoose library (mongoose.c +
# mongoose.h). These two files are dropped in verbatim from the upstream
# amalgamated release — cesanta distributes mongoose as a single-file
# library, so vendoring (not submoduling) matches their intended model.
#
# See ../update_mongoose.sh to verify or upgrade.
#
# Upstream:   https://github.com/cesanta/mongoose
# License:    GPL-2.0-only OR commercial (see file headers)

tag                = $target_tag
commit             = $commit_sha
pulled_at          = $today
sha256_mongoose_c  = $new_sha_c
sha256_mongoose_h  = $new_sha_h
EOF

printf 'updated src/mongoose.{c,h} and src/MONGOOSE_VERSION to %s\n' \
    "$target_tag"
printf 'review with: git diff -- src/mongoose.c src/mongoose.h src/MONGOOSE_VERSION\n'
