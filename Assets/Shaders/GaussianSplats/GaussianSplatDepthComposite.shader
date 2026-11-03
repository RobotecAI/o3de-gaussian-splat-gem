{
    "Source": "GaussianSplatDepthComposite.azsl",
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
        "BlendDest": "One",
        "BlendOp": "Minimum"
    },
    "DrawList": "forward",
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
