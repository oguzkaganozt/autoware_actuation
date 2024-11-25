group "default" {
  targets = [
    "simulator-visualizer",
    "planning-control-fail",
    "planning-control-pass"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-simulator-visualizer" {}
target "docker-metadata-action-planning-control-fail" {}
target "docker-metadata-action-planning-control-pass" {}

target "simulator-visualizer" {
  inherits = ["docker-metadata-action-simulator-visualizer"]
  dockerfile = "docker/simulator-visualizer/Dockerfile"
  target = "simulator-visualizer"
}

target "planning-control-fail" {
  inherits = ["docker-metadata-action-planning-control-fail"]
  dockerfile = "docker/planning-control/fail/Dockerfile"
  target = "planning-control-fail"
}

target "planning-control-pass" {
  inherits = ["docker-metadata-action-planning-control-pass"]
  dockerfile = "docker/planning-control/pass/Dockerfile"
  target = "planning-control-pass"
}
