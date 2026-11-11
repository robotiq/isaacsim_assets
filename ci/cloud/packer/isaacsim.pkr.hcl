packer {
  required_plugins {
    googlecompute = {
      source  = "github.com/hashicorp/googlecompute"
      version = ">= 1.1.0"
    }
  }
}

variable "project" {
  type        = string
  description = "GCP project the image is built and stored in (the launcher's --project)."
}

variable "owner" {
  type        = string
  description = "Value of the owner label on the build VM and the image."
}

variable "zone" {
  type    = string
  default = "us-central1-b"

  validation {
    condition     = contains(["us-central1-b", "us-central1-a", "us-central1-c", "us-central1-f"], var.zone)
    error_message = "Zone must be us-central1-b, us-central1-a, us-central1-c or us-central1-f, the us-central1 zones with T4 GPUs."
  }
}

locals {
  labels = {
    purpose = "isaac-runtime-stamp"
    owner   = var.owner
  }
}

source "googlecompute" "isaacsim" {
  project_id                      = var.project
  zone                            = var.zone
  source_image_family             = "common-cu129-ubuntu-2204-nvidia-580"
  source_image_project_id         = ["deeplearning-platform-release"]
  machine_type                    = "n1-standard-8"
  accelerator_type                = "projects/${var.project}/zones/${var.zone}/acceleratorTypes/nvidia-tesla-t4"
  accelerator_count               = 1
  on_host_maintenance             = "TERMINATE"
  network                         = "isaac-runtime-stamp"
  subnetwork                      = "isaac-runtime-stamp-us-central1"
  tags                            = ["isaac-runtime-stamp"]
  disk_size                       = 150
  use_os_login                    = false
  use_iap                         = true
  disable_default_service_account = true
  ssh_username                    = "packer"
  image_family                    = "isaacsim-6-2"
  image_name                      = "isaacsim-6-2-{{timestamp}}"
  state_timeout                   = "30m"
  image_labels                    = local.labels
  labels                          = local.labels
}

build {
  sources = ["source.googlecompute.isaacsim"]

  provisioner "shell" {
    script = "${path.root}/provision.sh"
  }
}
