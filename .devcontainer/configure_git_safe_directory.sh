#!/bin/bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v git >/dev/null 2>&1; then
    echo "git is not installed; skipping safe.directory setup." >&2
    exit 0
fi

if [ ! -d "$REPO_ROOT/.git" ] && [ ! -f "$REPO_ROOT/.git" ]; then
    echo "Not a git repository: $REPO_ROOT" >&2
    exit 0
fi

add_safe_directory() {
    local path="$1"

    if [ ! -e "$path" ]; then
        return 0
    fi

    if git config --global --get-all safe.directory | grep -Fxq "$path"; then
        return 0
    fi

    git config --global --add safe.directory "$path"
}

add_safe_directory "$REPO_ROOT"

if [ -f "$REPO_ROOT/.gitmodules" ]; then
    while read -r submodule_path; do
        [ -n "$submodule_path" ] || continue
        add_safe_directory "$REPO_ROOT/$submodule_path"
    done < <(git config --file "$REPO_ROOT/.gitmodules" --get-regexp '^submodule\..*\.path$' | awk '{print $2}')
fi

echo "Configured git safe.directory for this workspace."
