import sys, os
venv_site = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.venv/lib/python3.10/site-packages"))
if venv_site in sys.path:
    sys.path.remove(venv_site)
sys.path.insert(0, venv_site)
