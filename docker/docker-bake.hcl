group "default" {
  targets = [
    "aws-reinvent-simulator-monolithic"
  ]
}
target "docker-metadata-action-aws-reinvent-simulator-monolithic" {}

target "aws-reinvent-simulator-monolithic" {
  inherits = ["docker-metadata-action-aws-reinvent-simulator-monolithic"]
  dockerfile = "docker/Dockerfile"
  target = "aws-reinvent-simulator-monolithic"
}
