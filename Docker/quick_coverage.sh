#!/bin/bash
# Quick coverage generation for both Python and C++ packages

set -e
cd /workspace

echo "=========================================="
echo "  Code Coverage Report Generation"
echo "=========================================="
echo ""

# Python coverage
echo "📊 Python Coverage (simple_py)"
echo "------------------------------------------"
PY_OUTPUT=$(colcon test --packages-select simple_py --event-handlers console_direct+ --pytest-args --cov=simple_py --cov-report=term --cov-report=html --cov-report=lcov 2>&1)
echo "$PY_OUTPUT" | grep -B1 -A 10 "coverage: platform" | grep -v "^--$"

echo ""
echo "📊 C++ Coverage (simple_cpp)"
echo "------------------------------------------"
cd build/simple_cpp

# Check if coverage data exists
if ! find . -name "*.gcda" -type f | grep -q .; then
    echo "⚠️  No coverage data found!"
    echo "   C++ package needs to be built with coverage flags."
    echo ""
    echo "   To enable C++ coverage, rebuild with:"
    echo "   colcon build --packages-select simple_cpp --cmake-clean-cache \\"
    echo "     --cmake-args -DCMAKE_CXX_FLAGS='--coverage' \\"
    echo "                  -DCMAKE_C_FLAGS='--coverage' \\"
    echo "                  -DCMAKE_EXE_LINKER_FLAGS='--coverage'"
    echo ""
    echo "   Then run tests: colcon test --packages-select simple_cpp"
    cd /workspace
else
    # Generate coverage report
    lcov --capture --directory . --output-file coverage.info --ignore-errors mismatch,inconsistent >/dev/null 2>&1
    lcov --remove coverage.info '/usr/*' '*/test/*' '*/build/*' '*/gtest/*' '*/rosidl_*' --output-file coverage_filtered.info --ignore-errors unused >/dev/null 2>&1
    genhtml coverage_filtered.info --output-directory coverage_html --quiet 2>/dev/null
    
    echo "Project Source Files:"
    lcov --list coverage_filtered.info 2>/dev/null | grep -E "workspace/src/simple_cpp" -A1 | grep -E "(\.cpp|\.hpp)" | sed 's/^/  /'
    cd /workspace
fi

echo ""
echo "=========================================="
echo "✅ Coverage Reports Generated!"
echo "=========================================="
echo ""
echo "📁 HTML Reports:"
echo "   Python:  src/simple_py/htmlcov/index.html"
if [ -f "/workspace/build/simple_cpp/coverage_html/index.html" ]; then
    echo "   C++:     build/simple_cpp/coverage_html/index.html"
else
    echo "   C++:     (not available - see instructions above)"
fi
echo ""