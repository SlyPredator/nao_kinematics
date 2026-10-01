"""Master verification script for NAOKinematics Python port."""

import sys
from Python.tests.test_parity_h21 import test_h21_parity
from Python.tests.test_parity_h25 import test_h25_parity


def run_all():
    print("=" * 60)
    print("Running NAOKinematics Parity & Validation Suite")
    print("=" * 60)

    print("\n--- 1. Testing NAOH21 Parity against C++ ---")
    try:
        test_h21_parity()
    except Exception as e:
        print(f"FAILED NAOH21: {e}")
        sys.exit(1)

    print("\n--- 2. Testing NAOH25 Parity against C++ ---")
    try:
        test_h25_parity()
    except Exception as e:
        print(f"FAILED NAOH25: {e}")
        sys.exit(1)

    print("\n" + "=" * 60)
    print("ALL VERIFICATIONS AND PARITY TESTS COMPLETED SUCCESSFULLY!")
    print("=" * 60)


if __name__ == "__main__":
    run_all()
