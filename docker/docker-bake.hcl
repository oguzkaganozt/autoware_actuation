group "default" {
  targets = [
    "aws-reinvent-simulator-devel",
    "aws-reinvent-simulator",
    "aws-reinvent-planning-control"
  ]
}

target "docker-metadata-action-aws-reinvent-simulator-devel" {}
target "docker-metadata-action-aws-reinvent-simulator" {}
target "docker-metadata-action-aws-reinvent-planning-control" {}

target "aws-reinvent-simulator-devel" {
  inherits = ["docker-metadata-action-aws-reinvent-simulator-devel"]
  dockerfile = "docker/Dockerfile"
  target = "aws-reinvent-simulator-devel"
}

target "aws-reinvent-simulator" {
  inherits = ["docker-metadata-action-aws-reinvent-simulator"]
  dockerfile = "docker/Dockerfile"
  target = "aws-reinvent-simulator"
}

target "aws-reinvent-planning-control" {
  inherits = ["docker-metadata-action-aws-reinvent-planning-control"]
  dockerfile = "docker/Dockerfile"
  target = "aws-reinvent-planning-control"
}
