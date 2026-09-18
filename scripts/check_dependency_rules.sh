#!/usr/bin/env bash
# Build brief §2 "Dependency rules, enforced in CI":
#   1. schemas/ depends on nothing else in the monorepo.
#   2. No package anywhere compiles against a hand-written copy of a
#      generated type. Grep for the message names outside schemas/gen/ and
#      fail on a match in a non-generated file.
#   3. The kernel core links no ROS symbol. Check the symbol table.
#
# Run from the monorepo root: ./edge/scripts/check_dependency_rules.sh [build-dir]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

FAIL=0

echo "== rule 1: schemas/ depends on nothing else in the monorepo"
# schemas/*.proto and schemas/buf.* only ever reference other schemas/ files
# (relative "harness/v1/..." imports resolve within the buf module); nothing
# there should reach into backend/, edge/, sim/, or web/.
if grep -rn '\.\./\(backend\|edge\|sim\|web\)' schemas/proto schemas/buf.yaml schemas/buf.gen.yaml 2>/dev/null; then
  echo "FAIL: schemas/ references another monorepo package" >&2
  FAIL=1
else
  echo "  clean"
fi

echo "== rule 2: no hand-written copy of a generated message"
# Every message name declared in schemas/proto/. A hand-written struct/class/
# interface with one of these exact names outside schemas/gen/ is exactly the
# drift build brief §12 warns about ("a second definition... it will drift").
MESSAGES=$(grep -hoE '^message [A-Za-z0-9_]+' schemas/proto/harness/v1/*.proto | awk '{print $2}' | sort -u)

# path:Name pairs that are coincidental collisions with an unrelated,
# pre-existing domain concept, not a hand-written copy of the schema type.
# Add here only with a comment explaining why it's a false positive; anything
# added without one should be treated as a rule 2 violation on sight.
ALLOWLIST='
backend/app/embodiments.py:Embodiment   # pre-existing product-catalog model for the web registry (URDF/asset metadata) — unrelated to the Embodiment in profile.proto (joint order + DOF for the enforcement path)
'

for name in $MESSAGES; do
  # Look for a hand-authored type declaration with this name: a struct/class
  # (C++), a class (Python), or an interface/type (TypeScript) — outside
  # schemas/gen/ and outside this script itself.
  MATCHES=$(grep -rnE "^\s*(struct|class|interface|type) $name\b" \
    --include='*.cpp' --include='*.hpp' --include='*.h' --include='*.py' --include='*.ts' --include='*.tsx' \
    backend edge/src edge/include edge/tools sim app/src 2>/dev/null || true)
  while IFS= read -r match; do
    [ -z "$match" ] && continue
    file="${match%%:*}"
    if echo "$ALLOWLIST" | grep -q "^$file:$name"; then
      continue
    fi
    echo "FAIL: hand-written '$name' found outside schemas/gen/:" >&2
    echo "$match" >&2
    FAIL=1
  done <<< "$MATCHES"
done
[ "$FAIL" -eq 0 ] && echo "  clean (checked: $(echo "$MESSAGES" | tr '\n' ' '))"

echo "== rule 3: kernel core links no ROS symbol"
BUILD_DIR="${1:-edge/build}"
LIB=""
for candidate in "$BUILD_DIR"/libharness_kernel.a "$BUILD_DIR"/libharness_kernel.so "$BUILD_DIR"/libharness_kernel*.dylib; do
  [ -f "$candidate" ] && LIB="$candidate" && break
done
if [ -z "$LIB" ]; then
  echo "  skipped: no built libharness_kernel under $BUILD_DIR (build edge/ first to run this check)"
elif command -v nm >/dev/null 2>&1; then
  ROS_SYMS=$(nm "$LIB" 2>/dev/null | grep -iE 'rclcpp|ros2_control|ament_' || true)
  if [ -n "$ROS_SYMS" ]; then
    echo "FAIL: ROS symbol(s) found in $LIB:" >&2
    echo "$ROS_SYMS" >&2
    FAIL=1
  else
    echo "  clean ($LIB)"
  fi
else
  echo "  skipped: nm not available"
fi

exit "$FAIL"
