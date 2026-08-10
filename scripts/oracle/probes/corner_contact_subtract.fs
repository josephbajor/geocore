FeatureScript __LIBRARY_VERSION__;
import(path : "onshape/std/geometry.fs", version : "__LIBRARY_VERSION__.0");

annotation { "Feature Type Name" : "Corner contact subtract oracle" }
export const cornerContactSubtract = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        fCylinder(context, id + "firstCylinder", {
                    "bottomCenter" : vector(0, 0, 16) * meter,
                    "topCenter" : vector(0, 0, 17) * meter,
                    "radius" : 13 * meter
                });
        fCylinder(context, id + "secondCylinder", {
                    "bottomCenter" : vector(-14, 0, 0) * meter,
                    "topCenter" : vector(5, 0, 0) * meter,
                    "radius" : 20 * meter
                });
        opBoolean(context, id + "subtract", {
                    "targets" : qCreatedBy(id + "firstCylinder", EntityType.BODY),
                    "tools" : qCreatedBy(id + "secondCylinder", EntityType.BODY),
                    "operationType" : BooleanOperationType.SUBTRACTION,
                    "keepTools" : false
                });
    });
