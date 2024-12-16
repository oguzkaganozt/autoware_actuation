group "default" {
  targets = [
    "simulator",
    "planning-control"
  ]
}

target "docker-metadata-action-simulator" {}
target "docker-metadata-action-planning-control" {}

target "simulator" {
  inherits = ["docker-metadata-action-simulator"]
  dockerfile = "docker/Dockerfile"
  target = "simulator"
}

target "planning-control" {
  inherits = ["docker-metadata-action-planning-control"]
  dockerfile = "docker/Dockerfile"
  target = "planning-control"
}

