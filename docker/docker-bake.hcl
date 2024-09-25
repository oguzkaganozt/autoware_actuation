group "default" {
  targets = [
    "simulator-monolithic"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-simulator-monolithic" {}

target "simulator-monolithic-devel" {
  inherits = ["docker-metadata-action-simulator-monolithic"]
  dockerfile = "docker/Dockerfile"
  target = "simulator-monolithic"
}
