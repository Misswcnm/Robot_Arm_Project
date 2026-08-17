schema_add_parkings = {
  "type" : "object",
  "required": ["parkings"],
  "properties" : {
    "parkings" : {
      "type" : "array",
      "items": {
        "type": "object",
        "required": ["id","height","width","gridX","gridY","angle"],
        "properties": {
          "id": {"type":"string"},
          "height": {"type":"number"},
          "width": {"type":"number"},
          "gridX": {"type":"integer"},
          "gridX": {"type":"integer"},
          "angle": {"type":"number"}
        }
      }
    }
  }
}
schema_update_parkings = {
  "type": "object",
  "required": ["id","height","width","gridX","gridY","angle"],
  "properties": {
    "id": {"type":"string"},
    "height": {"type":"number"},
    "width": {"type":"number"},
    "gridX": {"type":"integer"},
    "gridX": {"type":"integer"},
    "angle": {"type":"number"}
  }
}
schema_delete_parkings = {
  "type" : "object",
  "required": ["id"],
  "properties" : {
    "id" : {
      "type" : "array",
      "items": {
        "type": "string"
        }
      }
  }
}
schema_add_area = {
  "type": "object",
  "required": ["name","polygon"],
  "properties": {
    "name": {"type": "string"},
    "polygon": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["x", "y"],
        "properties": {
          "x": {"type": "number"},
          "y": {"type": "number"}
        }
      }
    }
  }
}
schema_delete_area = {
  "type" : "object",
  "required": ["name"],
  "properties" : {
    "name" : {"type" : "string"}
  }
}
