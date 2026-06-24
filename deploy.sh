#!/usr/bin/env bash
# Deploy by making the robot check out YOUR current branch via git, so the
# device only ever runs tracked, committed, pushed code — nothing untracked
# ever lands on it.
#
# Usage:
#   ./deploy.sh                 # push current branch + sync robot to it
#   ./deploy.sh run maze.py     # ...then run a script on the robot
#
# Override the target with env vars:
#   PICAR_HOST=pi@192.168.1.42 PICAR_DIR=~/picar ./deploy.sh
set -euo pipefail

PICAR_HOST="${PICAR_HOST:-pi@picar.local}"
PICAR_DIR="${PICAR_DIR:-~/picar}"
REMOTE="${PICAR_GIT_REMOTE:-origin}"

BRANCH="$(git rev-parse --abbrev-ref HEAD)"

# A commit is the unit of deploy. Uncommitted edits won't ship — say so loudly
# rather than silently running stale code on the robot.
if [[ -n "$(git status --porcelain)" ]]; then
  echo "!! You have uncommitted changes — they will NOT be deployed:"
  git status --short
  echo "   Commit them first (even a WIP commit), then re-run ./deploy.sh"
  exit 1
fi

echo ">> Pushing ${BRANCH} to ${REMOTE}"
git push "${REMOTE}" "${BRANCH}"

echo ">> Syncing ${PICAR_HOST}:${PICAR_DIR} to ${REMOTE}/${BRANCH}"
ssh "${PICAR_HOST}" "
  set -e
  cd ${PICAR_DIR}
  git fetch ${REMOTE}
  git checkout ${BRANCH}
  git reset --hard ${REMOTE}/${BRANCH}
  git clean -fd        # remove any untracked files left on the device
"

if [[ "${1:-}" == "run" ]]; then
  shift
  echo ">> Running on robot: python3 $*"
  # shellcheck disable=SC2029
  ssh -t "${PICAR_HOST}" "cd ${PICAR_DIR} && python3 $*"
fi
