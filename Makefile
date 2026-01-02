# N2K Simulator Pro - Makefile
# Author: Colin Bitterfield
# Email: colin@bitterfield.com
# Version: 1.0.0

.PHONY: help build run stop restart logs status clean docker-push docker-login test

# Docker image configuration
DOCKER_USER ?= colinbitterfield
IMAGE_NAME = n2k-simulator-pro
VERSION ?= $(shell cat VERSION 2>/dev/null || echo "1.0.0")
DOCKER_IMAGE = $(DOCKER_USER)/$(IMAGE_NAME):$(VERSION)
DOCKER_IMAGE_LATEST = $(DOCKER_USER)/$(IMAGE_NAME):latest

# Colors for output
BLUE := \033[0;36m
GREEN := \033[0;32m
YELLOW := \033[0;33m
RED := \033[0;31m
NC := \033[0m

help: ## Show this help message
	@echo "$(BLUE)N2K Simulator Pro$(NC)"
	@echo "$(GREEN)Available targets:$(NC)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  $(YELLOW)%-20s$(NC) %s\n", $$1, $$2}'

build: ## Build Docker image
	@echo "$(BLUE)Building Docker image...$(NC)"
	docker-compose build
	@echo "$(GREEN)Build complete!$(NC)"

run: ## Run simulator in foreground
	@echo "$(BLUE)Starting N2K Simulator Pro...$(NC)"
	docker-compose up
	@echo "$(GREEN)Simulator running at http://192.168.68.51:9001$(NC)"

daemon: ## Run simulator in background
	@echo "$(BLUE)Starting N2K Simulator Pro in background...$(NC)"
	docker-compose up -d
	@echo "$(GREEN)Simulator running at http://192.168.68.51:9001$(NC)"

stop: ## Stop simulator
	@echo "$(YELLOW)Stopping N2K Simulator Pro...$(NC)"
	docker-compose stop

restart: stop daemon ## Restart simulator

logs: ## Show simulator logs
	@echo "$(BLUE)Showing logs (Ctrl+C to exit)...$(NC)"
	docker-compose logs -f n2k-simulator

status: ## Check simulator status
	@echo "$(BLUE)Checking N2K Simulator status...$(NC)"
	@curl -s http://192.168.68.51:9001/api/status | python3 -m json.tool || echo "$(RED)Simulator not running$(NC)"

device-info: ## Show NMEA 2000 device information
	@echo "$(BLUE)N2K Device Information:$(NC)"
	@curl -s http://192.168.68.51:9001/api/device | python3 -m json.tool

heading-info: ## Show compass heading configuration
	@echo "$(BLUE)Compass Heading Configuration:$(NC)"
	@curl -s http://192.168.68.51:9001/api/heading | python3 -m json.tool

start-gps: ## Start GPS broadcast
	@echo "$(BLUE)Starting GPS broadcast...$(NC)"
	@curl -X POST http://192.168.68.51:9001/api/start
	@echo ""

stop-gps: ## Stop GPS broadcast
	@echo "$(YELLOW)Stopping GPS broadcast...$(NC)"
	@curl -X POST http://192.168.68.51:9001/api/stop
	@echo ""

set-drift: ## Set drift mode (anchor drag simulation)
	@echo "$(BLUE)Setting drift mode...$(NC)"
	@curl -X POST http://192.168.68.51:9001/api/mode \
		-H "Content-Type: application/json" \
		-d '{"mode":"drift"}'
	@echo ""

set-anchoring: ## Set anchoring mode (realistic anchor swing)
	@echo "$(BLUE)Setting anchoring mode...$(NC)"
	@curl -X POST http://192.168.68.51:9001/api/mode \
		-H "Content-Type: application/json" \
		-d '{"mode":"anchoring"}'
	@echo ""

set-stationary: ## Set stationary mode
	@echo "$(BLUE)Setting stationary mode...$(NC)"
	@curl -X POST http://192.168.68.51:9001/api/mode \
		-H "Content-Type: application/json" \
		-d '{"mode":"stationary"}'
	@echo ""

test-tcp: ## Test TCP NMEA stream
	@echo "$(BLUE)Testing TCP NMEA stream (Ctrl+C to exit)...$(NC)"
	@nc 192.168.68.51 10110 | head -20

test-udp: ## Test UDP NMEA broadcast
	@echo "$(BLUE)Listening for UDP NMEA broadcast (Ctrl+C to exit)...$(NC)"
	@nc -u -l 2000

enable-can: ## Enable CAN bus output
	@echo "$(BLUE)Enabling CAN bus output...$(NC)"
	@curl -X POST http://192.168.68.51:9001/api/can/enable
	@echo ""

disable-can: ## Disable CAN bus output
	@echo "$(YELLOW)Disabling CAN bus output...$(NC)"
	@curl -X POST http://192.168.68.51:9001/api/can/disable
	@echo ""

enable-serial: ## Enable RS485 serial output
	@echo "$(BLUE)Enabling RS485 serial output...$(NC)"
	@curl -X POST http://192.168.68.51:9001/api/serial/enable
	@echo ""

disable-serial: ## Disable RS485 serial output
	@echo "$(YELLOW)Disabling RS485 serial output...$(NC)"
	@curl -X POST http://192.168.68.51:9001/api/serial/disable
	@echo ""

clean: ## Clean Docker containers and volumes
	@echo "$(YELLOW)Cleaning up...$(NC)"
	docker-compose down -v
	@echo "$(GREEN)Cleanup complete!$(NC)"

rebuild: clean build ## Clean and rebuild everything

docker-tag: ## Tag Docker image for Docker Hub
	@echo "$(BLUE)Tagging Docker image...$(NC)"
	docker tag anchor-drag-alarm-n2k-simulator $(DOCKER_IMAGE)
	docker tag anchor-drag-alarm-n2k-simulator $(DOCKER_IMAGE_LATEST)
	@echo "$(GREEN)Tagged: $(DOCKER_IMAGE)$(NC)"
	@echo "$(GREEN)Tagged: $(DOCKER_IMAGE_LATEST)$(NC)"

docker-login: ## Login to Docker Hub
	@echo "$(BLUE)Logging in to Docker Hub...$(NC)"
	docker login

docker-push: build docker-tag ## Build, tag, and push to Docker Hub
	@echo "$(BLUE)Pushing to Docker Hub...$(NC)"
	docker push $(DOCKER_IMAGE)
	docker push $(DOCKER_IMAGE_LATEST)
	@echo "$(GREEN)Pushed to Docker Hub:$(NC)"
	@echo "  $(DOCKER_IMAGE)"
	@echo "  $(DOCKER_IMAGE_LATEST)"

docker-pull: ## Pull image from Docker Hub
	@echo "$(BLUE)Pulling from Docker Hub...$(NC)"
	docker pull $(DOCKER_IMAGE_LATEST)

version: ## Show version
	@echo "$(BLUE)N2K Simulator Pro$(NC)"
	@echo "Version: $(VERSION)"

bump-version: ## Bump version (usage: make bump-version VERSION=1.0.1)
	@echo "$(VERSION)" > VERSION
	@echo "$(GREEN)Version updated to $(VERSION)$(NC)"

# Development targets
dev-install: ## Install Python dependencies for local development
	@echo "$(BLUE)Installing Python dependencies...$(NC)"
	cd n2k-simulator && pip install -r requirements.txt
	@echo "$(GREEN)Dependencies installed!$(NC)"

dev-run: ## Run simulator locally (without Docker)
	@echo "$(BLUE)Running simulator locally...$(NC)"
	cd n2k-simulator && python main.py

# Testing targets
test: ## Run tests
	@echo "$(BLUE)Running tests...$(NC)"
	@echo "$(YELLOW)Test suite not yet implemented$(NC)"

lint: ## Lint Python code
	@echo "$(BLUE)Linting Python code...$(NC)"
	flake8 n2k-simulator/*.py || echo "$(YELLOW)flake8 not installed$(NC)"

# Documentation targets
docs: ## Generate documentation
	@echo "$(BLUE)Generating documentation...$(NC)"
	@echo "$(YELLOW)Documentation generation not yet implemented$(NC)"

# Default target
.DEFAULT_GOAL := help
