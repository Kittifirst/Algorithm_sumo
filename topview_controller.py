"""Desktop closed-loop sumo controller. Run with --demo to try without hardware."""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="Start in simulation mode; never connect hardware")
    args = parser.parse_args()
    try:
        from PySide6 import QtWidgets
        from sumo.gui import MainWindow
    except ImportError as error:
        print(f"Missing dependency: {error}\nRun: .venv\\Scripts\\python.exe -m pip install -r requirements.txt", file=sys.stderr)
        return 1
    app = QtWidgets.QApplication(sys.argv[:1])
    window = MainWindow(demo=args.demo)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
