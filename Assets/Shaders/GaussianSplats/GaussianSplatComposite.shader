{
    "Source": "GaussianSplatComposite.azsl",
    "RasterState": {
        "CullMode": "Back"
    },
    "DepthStencilState": {
        "Depth": {
            "Enable": false
        }
    },
    "GlobalTargetBlendState": {
        "Enable": true,
        "BlendSource": "One",
        "BlendDest": "AlphaSourceInverse",
        "BlendOp": "Add"
    },
    "ProgramSettings": {
        "EntryPoints": [
            {
                "name": "MainVS",
                "type": "Vertex"
            },
            {
                "name": "MainPS",
                "type": "Fragment"
            }
        ]
    }
}
