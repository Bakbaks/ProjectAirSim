"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Global pytest configuration for ProjectAirSim tests.
Provides test isolation and stability improvements.
"""

import pytest
import time
import gc


@pytest.fixture(scope="function", autouse=True)
def test_isolation(request):
    """
    Automatic fixture to improve test isolation.
    
    Adds a delay after each test to allow the simulator to settle,
    preventing race conditions and state pollution between sequential tests.
    This is especially important for tests that:
    - Load/unload scenes
    - Modify textures/materials
    - Control drone movements
    - Capture images
    
    The delay can be skipped by marking a test with @pytest.mark.no_isolation
    """
    # Before test: force garbage collection to clean up any lingering objects
    gc.collect()
    
    yield
    
    # Check if test is marked to skip isolation delay
    if "no_isolation" in request.keywords:
        return
    
    # After test: add delay to allow simulator to settle
    # This prevents issues with:
    # - Scene loading timeouts
    # - Texture/material update propagation
    # - Image capture timing
    time.sleep(1.5)
    
    # Force garbage collection after test
    gc.collect()


@pytest.fixture(scope="session", autouse=True)
def session_setup_teardown():
    """
    Session-level setup and teardown.
    Runs once before all tests and once after all tests complete.
    """
    print("\n" + "="*80)
    print("Starting ProjectAirSim test session")
    print("Tests will run sequentially with isolation delays")
    print("="*80 + "\n")
    
    yield
    
    print("\n" + "="*80)
    print("ProjectAirSim test session complete")
    print("="*80 + "\n")


@pytest.fixture(scope="module")
def module_isolation():
    """
    Module-level isolation fixture.
    Add longer delay between test modules to allow major state resets.
    """
    yield
    # Longer delay between modules
    print("\n--- Module complete, allowing simulator to reset ---")
    time.sleep(3.0)


# Custom pytest markers for test categorization
def pytest_configure(config):
    """Register custom markers for test categorization."""
    config.addinivalue_line(
        "markers", "no_isolation: skip automatic test isolation delay"
    )
    config.addinivalue_line(
        "markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')"
    )
    config.addinivalue_line(
        "markers", "requires_scene(name): marks tests that require specific scene loaded"
    )


def pytest_runtest_makereport(item, call):
    """
    Hook to track test failures and potentially adjust isolation behavior.
    """
    if call.when == "call":
        # Could add custom logic here to increase delays after failures
        pass

