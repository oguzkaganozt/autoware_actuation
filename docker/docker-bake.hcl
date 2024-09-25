group "default" {
  targets = [
    "aws-reinvent-simulator-monolithic-devel",
    "aws-reinvent-simulator-monolithic"
  ]
}

target "docker-metadata-action-aws-reinvent-simulator-monolithic-devel" {}
target "docker-metadata-action-aws-reinvent-simulator-monolithic" {}

target "aws-reinvent-simulator-monolithic-devel" {
  inherits = ["docker-metadata-action-aws-reinvent-simulator-monolithic-devel"]
  dockerfile = "docker/Dockerfile"
  target = "aws-reinvent-simulator-monolithic-devel"
}

target "aws-reinvent-simulator-monolithic" {
  inherits = ["docker-metadata-action-aws-reinvent-simulator-monolithic"]
  dockerfile = "docker/Dockerfile"
  target = "aws-reinvent-simulator-monolithic"
}
