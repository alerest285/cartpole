#!/bin/zsh
cd "${0:A:h}" || exit 1
exec .venv/bin/python -m cartpole serve --backend sim --port 8000 --demo
