{
    "Source": "GaussianSplatRadixOnesweep.azsl",
    "ProgramSettings": {
        "EntryPoints": [
            {
                "name": "MainCS",
                "type": "Compute"
            }
        ]
    },
    "AddBuildArguments": {
        "dxc": ["-fspv-target-env=vulkan1.1"]
    }
}
