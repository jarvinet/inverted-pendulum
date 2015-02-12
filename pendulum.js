/*


                       +-----+
                       |     |
     ==================+==+==+================
                       |  |  |
                       +--+--+
                          |
                          |
                          |
                          |
                          |
                          |
                          |
                          |
                          |
                       +--+--+
                       |  |  |
                       |  |  |
                       |     |
                       +-----+


 */


function init() {
    var   b2Vec2 = Box2D.Common.Math.b2Vec2
    ,  b2AABB = Box2D.Collision.b2AABB
    ,  b2BodyDef = Box2D.Dynamics.b2BodyDef
    ,  b2Body = Box2D.Dynamics.b2Body
    ,  b2FixtureDef = Box2D.Dynamics.b2FixtureDef
    ,  b2Fixture = Box2D.Dynamics.b2Fixture
    ,  b2World = Box2D.Dynamics.b2World
    ,  b2MassData = Box2D.Collision.Shapes.b2MassData
    ,  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
    ,  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
    ,  b2DebugDraw = Box2D.Dynamics.b2DebugDraw
    ,  b2MouseJointDef =  Box2D.Dynamics.Joints.b2MouseJointDef
    ,  b2RevoluteJointDef =  Box2D.Dynamics.Joints.b2RevoluteJointDef
    ,  b2PrismaticJointDef =  Box2D.Dynamics.Joints.b2PrismaticJointDef
    ;

    var world = new b2World(
        new b2Vec2(0, 10)    //gravity
        ,  true              //allow sleep
    );

    var memory = 0;

    var dt = 1/60;

    var setPointField = document.getElementById("setPoint")
    var impulseField = document.getElementById("impulse")
    var impulsePointField = document.getElementById("impulsePoint")
    var originalJointAngleField = document.getElementById("originalJointAngle")
    var jointAngleField = document.getElementById("jointAngle")
    var jointAngleSumField = document.getElementById("jointAngleSum")
    var jointAngleDerivativeField = document.getElementById("jointAngleDerivative")
    var jointAngleHistoryField = document.getElementById("jointAngleHistory")

    var errorField = document.getElementById("error")
    var errorsSumField = document.getElementById("errorsSum")
    var errorsDerivativeField = document.getElementById("errorsDerivative")
    var errorsHistoryField = document.getElementById("errorsHistory")

    var processVariables = [];
    var processVariablesMaxSize = 10; // max number of values to keep in processVariables
    var sumOfProcessVariables = 0; // sum of all stored process variables

    var error = 0;
    var errors = [];
    var errorsMaxSize = 10; // max number of values to keep in errorsprocessVariables
    var sumOfErrors = 0; // sum of all stored errors

    var setPoint = Math.PI;
    var TWO_PI = Math.PI*2;

    var origin = new b2Vec2(0,0);

    function impulseFunc(amount) {
        var impulseVec = new b2Vec2(amount, 0);
        var point = cartBody.GetWorldPoint(origin);
        cartBody.ApplyImpulse(impulseVec, point);
    }

    document.getElementById("buttonLeft").onclick = function () { impulseFunc(-1); };
    document.getElementById("buttonRight").onclick = function () { impulseFunc(1); }

    setPointField.innerText = setPoint;

    var fixDef = new b2FixtureDef;
    fixDef.density = 1.0;
    fixDef.friction = 0.5;
    fixDef.restitution = 0.2;

    var cartRailFixtureDef = new b2FixtureDef;
    cartRailFixtureDef.density = 1.0;
    cartRailFixtureDef.friction = 0.5;
    cartRailFixtureDef.restitution = 0.2;
    cartRailFixtureDef.shape = new b2PolygonShape;

    var cartFixtureDef = new b2FixtureDef;
    cartFixtureDef.density = 1.0;
    cartFixtureDef.friction = 0.5;
    cartFixtureDef.restitution = 0.2;
    cartFixtureDef.shape = new b2PolygonShape;

    var pendulumArmFixtureDef = new b2FixtureDef;
    pendulumArmFixtureDef.density = 1.0;
    pendulumArmFixtureDef.friction = 0.5;
    pendulumArmFixtureDef.restitution = 0.2;
    pendulumArmFixtureDef.shape = new b2PolygonShape;
    //pendulumArmFixtureDef.shape = new b2CircleShape;
    pendulumArmFixtureDef.filter.maskBits = 0x0000; // disable collision with other bodies

    var bodyDef = new b2BodyDef;
    var cartRailBodyDef = new b2BodyDef;
    var cartBodyDef = new b2BodyDef;
    var pendulumBodyDef = new b2BodyDef;

    //create ground
    bodyDef.type = b2Body.b2_staticBody;
    fixDef.shape = new b2PolygonShape;
    fixDef.shape.SetAsBox(20, 2);
    bodyDef.position.Set(10, 400 / 30 + 1.8);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    bodyDef.position.Set(10, -1.8);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    fixDef.shape.SetAsBox(2, 14);
    bodyDef.position.Set(-1.8, 13);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    bodyDef.position.Set(21.8, 13);
    world.CreateBody(bodyDef).CreateFixture(fixDef);

    //create some objects
    bodyDef.type = b2Body.b2_staticBody;
    fixDef.shape = new b2PolygonShape;

    fixDef.shape.SetAsBox(2, 14);
    bodyDef.position.Set(-1.8, 13);
    world.CreateBody(bodyDef).CreateFixture(fixDef);

    // cart rail
    cartRailBodyDef.type = b2Body.b2_staticBody;
    cartRailFixtureDef.shape.SetAsBox(5, 0.1);
    cartRailBodyDef.position.Set(10, 7);
    var cartRailBody = world.CreateBody(cartRailBodyDef);
    cartRailBody.CreateFixture(cartRailFixtureDef)

    // cart
    cartBodyDef.type = b2Body.b2_dynamicBody;
    cartFixtureDef.shape.SetAsBox(0.5, 0.5);
    cartBodyDef.position.Set(10, 7);
    var cartBody = world.CreateBody(cartBodyDef);
    cartBody.CreateFixture(cartFixtureDef)

    // joint between cart and cart rail
    var prismaticJointDef = new b2PrismaticJointDef();
    prismaticJointDef.bodyA = cartRailBody;
    prismaticJointDef.bodyB = cartBody;
    prismaticJointDef.lowerTranslation = -5;
    prismaticJointDef.upperTranslation = 5;
    prismaticJointDef.enableLimit = true;
    prismaticJointDef.localAnchorA = new b2Vec2(0,0);
    prismaticJointDef.localAnchorB = new b2Vec2(0,0);
    prismaticJointDef.localAxisA = new b2Vec2(1,0);
    var prismaticJoint = world.CreateJoint(prismaticJointDef);

    // pendulum
    pendulumBodyDef.type = b2Body.b2_dynamicBody;
    pendulumBodyDef.position.Set(10, 7);
    var pendulumBody = world.CreateBody(pendulumBodyDef);
    pendulumArmFixtureDef.shape.SetAsBox(1.0, 1.0);
    pendulumBody.CreateFixture(pendulumArmFixtureDef)

    // joint between cart and pendulum
    var revoluteJointDef = new b2RevoluteJointDef();
    revoluteJointDef.bodyA = pendulumBody;
    revoluteJointDef.bodyB = cartBody;
    revoluteJointDef.enableLimit = false;
    revoluteJointDef.localAnchorA = new b2Vec2(0,-4);
    revoluteJointDef.localAnchorB = new b2Vec2(0,0);
    revoluteJointDef.localAxisA = new b2Vec2(1,0);
    var revoluteJoint = world.CreateJoint(revoluteJointDef);

    //setup debug draw
    var debugDraw = new b2DebugDraw();
    debugDraw.SetSprite(document.getElementById("canvas").getContext("2d"));
    debugDraw.SetDrawScale(30.0);
    debugDraw.SetFillAlpha(0.5);
    debugDraw.SetLineThickness(1.0);
    debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
    world.SetDebugDraw(debugDraw);

    window.setInterval(update, 1000 / 60);

    //mouse

    var mouseX, mouseY, mousePVec, isMouseDown, selectedBody, mouseJoint;
    var canvasPosition = getElementPosition(document.getElementById("canvas"));

    document.addEventListener("mousedown", function(e) {
        isMouseDown = true;
        handleMouseMove(e);
        document.addEventListener("mousemove", handleMouseMove, true);
    }, true);

    document.addEventListener("mouseup", function() {
        document.removeEventListener("mousemove", handleMouseMove, true);
        isMouseDown = false;
        mouseX = undefined;
        mouseY = undefined;
    }, true);

    function handleMouseMove(e) {
        mouseX = (e.clientX - canvasPosition.x) / 30;
        mouseY = (e.clientY - canvasPosition.y) / 30;
    };

    function getBodyAtMouse() {
        mousePVec = new b2Vec2(mouseX, mouseY);
        var aabb = new b2AABB();
        aabb.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
        aabb.upperBound.Set(mouseX + 0.001, mouseY + 0.001);

        // Query the world for overlapping shapes.

        selectedBody = null;
        world.QueryAABB(getBodyCB, aabb);
        return selectedBody;
    }

    function getBodyCB(fixture) {
        if(fixture.GetBody().GetType() != b2Body.b2_staticBody) {
            if(fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)) {
                selectedBody = fixture.GetBody();
                return false;
            }
        }
        return true;
    }

    //update

    function update() {

        if(isMouseDown && (!mouseJoint)) {
            var body = getBodyAtMouse();

            if(body) {
                var md = new b2MouseJointDef();
                md.bodyA = world.GetGroundBody();
                md.bodyB = body;
                md.target.Set(mouseX, mouseY);
                md.collideConnected = true;
                md.maxForce = 300.0 * body.GetMass();
                mouseJoint = world.CreateJoint(md);
                body.SetAwake(true);
            }
        }

        var jointAngleOriginal = revoluteJoint.GetJointAngle();
        originalJointAngleField.innerText = jointAngleOriginal;

        var jointAngle = normalizeAngle(jointAngleOriginal);
        jointAngleField.innerText = jointAngle;

        var jointAngleSum = addProcessVariable(jointAngle);
        jointAngleSumField.innerText = jointAngleSum;

        var jointAngleDerivative = processVariablesDerivative();
        jointAngleDerivativeField.innerText = jointAngleDerivative;

//        jointAngleHistoryField.innerText = processVariables.join("\n");

        error = setPoint - jointAngle;
        errorField.innerText = error;

        var errorDerivative = errorsDerivative();
        errorsDerivativeField.innerText = errorDerivative;

        var errorsSum = addError(error);
        errorsSumField.innerText = errorsSum;

//        errorsHistoryField.innerText = errors.join("\n");
        var engagePIDController = document.getElementById("engagePIDController").checked;
        if (engagePIDController) {
            var proportionalGain = parseInt(document.getElementById("proportionalGain").innerHTML) / 1000;
            var integralGain = parseInt(document.getElementById("integralGain").innerHTML) / 1000;
            var derivativeGain = parseInt(document.getElementById("derivativeGain").innerHTML) / 1000;

            var impulse = (proportionalGain*error + integralGain*errorsSum + derivativeGain*errorDerivative)/1;
            if (!isNaN(impulse)) {
                var impulseVec = new b2Vec2(impulse,0);
                var point = cartBody.GetWorldPoint(origin);
                impulseField.innerText = impulse;
                impulsePointField.innerText = point.x + " " + point.y;
                cartBody.ApplyImpulse(impulseVec, point);
            }
        }
        //alert(field);
        if(mouseJoint) {
            if(isMouseDown) {
                mouseJoint.SetTarget(new b2Vec2(mouseX, mouseY));
            } else {
                world.DestroyJoint(mouseJoint);
                mouseJoint = null;
            }
        }

        world.Step(dt, 10, 10);
        world.DrawDebugData();
        world.ClearForces();
    };

    //helpers

    function normalizeAngle(angle) {
        while (angle > TWO_PI) {
            angle -= TWO_PI;
        }
        while (angle < 0) {
            angle += TWO_PI;
        }
        return angle;
    }

    function addProcessVariable(value) {
        processVariables.push(value);
        sumOfProcessVariables += value;
        if (processVariables.length > processVariablesMaxSize) {
            var removedValue = processVariables.shift();
            sumOfProcessVariables -= removedValue;
        }
        return sumOfProcessVariables;
    }

    function processVariablesDerivative() {
        return (processVariables[processVariablesMaxSize-1] - processVariables[processVariablesMaxSize-2]) / dt;
    }

    function addError(value) {
        errors.push(value);
        sumOfErrors += value;
        if (errors.length > errorsMaxSize) {
            var removedValue = errors.shift();
            sumOfErrors -= removedValue;
        }
        return sumOfErrors;
    }

    function errorsDerivative() {
        return (errors[errorsMaxSize-1] - errors[errorsMaxSize-2]) / dt;
    }

    //http://js-tut.aardon.de/js-tut/tutorial/position.html
    function getElementPosition(element) {
        var elem=element, tagname="", x=0, y=0;

        while((typeof(elem) == "object") && (typeof(elem.tagName) != "undefined")) {
            y += elem.offsetTop;
            x += elem.offsetLeft;
            tagname = elem.tagName.toUpperCase();

            if(tagname == "BODY")
                elem=0;

            if(typeof(elem) == "object") {
                if(typeof(elem.offsetParent) == "object")
                    elem = elem.offsetParent;
            }
        }

        return {x: x, y: y};
    }
};
