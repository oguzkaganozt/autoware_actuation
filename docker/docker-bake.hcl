group "default" {
  targets = [
    "simulator-visualizer"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-simulator-visualizer" {}

target "simulator-visualizer" {
  inherits = ["docker-metadata-action-simulator-visualizer"]
  dockerfile = "docker/simulator-visualizer/Dockerfile"
  target = "simulator-visualizer"
}
