.PHONY: lint format test verify clean

lint: ## Run all project linters (Ruff + Flake8)
	ruff check ground_station/
	flake8 ground_station/ --max-line-length=130 --ignore=E203,W503,E402

format: ## Format code with Ruff
	ruff format ground_station/

test: ## Run ground station unit and integration test suite
	pytest ground_station/tests/ -v

verify-firmware: ## Compile and execute C++ firmware verification test
	g++ -std=c++17 -Wall -Wextra -Werror -I./firmware -I./firmware/common -I./firmware/airborne_fc/include firmware/test_firmware_math.cpp -o firmware/test_firmware_math
	./firmware/test_firmware_math

check: lint test verify-firmware ## Run all checks (lint, test, firmware)
