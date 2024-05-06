class Part {
    constructor (sphere, flatOrRound,
                 sphere2part, part2sphere, 
                 joint2parent, parent2joint, extraBodyPart) {
        // sphere defines the underlying part before distortion.
        this.sphere = sphere;
        
        // flatOrRound = 1:  flat shading.  = 2 : round shading.
        this.flatOrRound = flatOrRound;
        
        // sphere2part is the distortion of the sphere to make the part we want.
        this.sphere2part = sphere2part;
        this.part2sphere = part2sphere;
        
        // part2joint is the joint transformation.  Initially it is the
        // identity, meaning the joint is relaxed.  Changing part2joint
        // nods the head or bends the elbow.
        this.part2joint = new Mat();
        this.joint2part = new Mat();
        
        // joint2parent is the position of the joint with respect to the
        // parent when the joint is relaxed.  For example, the head is on
        // top of the body, not at its origin (center).
        this.joint2parent = joint2parent;
        this.parent2joint = parent2joint;
        
        // parent2world is the placement of the parent PART in the world.
        // (Not the parent sphere.)
        this.parent2world = new Mat();
        this.world2parent = new Mat();
        
        // sphere2world is the placement of the sphere in the world.  It
        // gets bound to model2world in the shading program.
        this.sphere2world = new Mat();
        this.world2sphere = new Mat();
        
        // sMin is the smallest positive s for a ray q + s u hitting a face
        // of the sphere.  It is Double.POSITIVE_INFINITY if no face is hit.
        this.sMin = Infinity;
        
        // pMin is the point on the sphere at which the ray hits closest.
        this.pMin = new PV(true);
        
        // children contains the child parts that are attached to this part.
        this.children = [];

        this.extra = extraBodyPart;

    }
        
    setParent2World (parent2world, world2parent) {
        this.parent2world = parent2world;
        this.world2parent = world2parent;
        
        // EXERCISE 1:
        // Set sphere2world and world2sphere.
        // HERE
        this.sphere2world = this.parent2world.times(this.joint2parent).times(this.part2joint).times(this.sphere2part);
        this.world2sphere = this.part2sphere.times(this.joint2part).times(this.parent2joint).times(this.world2parent);



        
        // Call setParent2world for each child.  Be careful!  You don't
        // just pass along parent2world and world2parent.
        // HERE
        var part2world = this.parent2world.times(this.joint2parent).times(this.part2joint);
        var world2part = this.joint2part.times(this.parent2joint).times(this.world2parent);


        
        for (var i = 0; i < this.children.length; i++)
            this.children[i].setParent2World(part2world, world2part);
    }

    // EXERCISE 2:
    render (gl, program) {
        // Setting model2world in shader.  (The sphere is the model.)
        gl.uniformMatrix4fv(gl.getUniformLocation( program, "model2world" ), 
                            false, this.sphere2world.flatten());
        
        // Set world2modelT (T means transpose) in shader.
	// Fix robot.html so it uses world2modelT where it should.
        // HERE
        gl.uniformMatrix4fv(gl.getUniformLocation( program, "world2modelT"), false, this.world2sphere.transpose().flatten());


        
        // Render this part's sphere.
        this.sphere.render(gl, program, this.flatOrRound);
        
        // Recursively call render for each child.
        // HERE
        for (var i = 0; i < this.children.length; i++){
            this.children[i].render(gl, program, this.flatOrRound);
        }


        
        // TEST IT
    }
    

    // EXERCISE 7:
    // Determine part that is hit closest to front on the line segment
    // from front to back in world coordinates.  Set sMin and pMin for
    // that part.  Update closest (properly) in the loop.
    closestHit (front, back) {
        // Calculate q and u in sphere coordinates and use sphere.closestHit.
        // HERE

        var q = this.world2sphere.times(front);
        var u = this.world2sphere.times(back.minus(front));
        this.sMin = this.sphere.closestHit(q, u);
        this.pMin = q.plus(u.times(this.sMin));


        console.log("q " + q);
        console.log("u " + u);
        console.log("this.sMin " + this.sMin);
        
        var closest = null;
        if (this.sMin != Infinity)
            closest = this;
        
        // Recurse for each child.  Take the closest hit of all.
        for (var i = 0; i < this.children.length; i++) {
            var hit = this.children[i].closestHit(front, back);
            
            // Update closest.  Remember, if both hit and closest are
            // not null, you need to compare hit.sMin and closest.sMin.
            // HERE
            if (hit != null){
                if (closest != null){
                    if (hit.sMin < closest.sMin){
                        closest = hit;
                    }
                }
                else {
                    closest = hit;
                }
            }
        }
            return closest;
    }
    
    // Drag part based on mouse position that corresponds to the line
    // segment from front to back in world coordinates.
    drag (front, back) {
        var o = new PV(true); // the origin in joint coordinates

        // EXERCISE 9A:
        // p is pMin in joint coordinates.
        // v is the vector from the origin to p.
        // f and b are front and back in joint coordinates.
        // Calculate of, fb, w, and the rotation of v into w.
        // Apply it to part2joint (which takes the place of model2rotated).
        var p = this.part2joint.times(this.sphere2part).times(this.pMin);
        var v = p.minus(o); 
        var f = this.parent2joint.times(this.world2parent).times(front);
        var b = this.parent2joint.times(this.world2parent).times(back);




        var of = f.minus(o);
        var fb = b.minus(f).unit();

        // (of + fb s) * fb = 0
        // of * fb + s = 0
        // s = - of * fb
        var s = -of.dot(fb);
        // w is the vector from the center to that closest point.
        var w = of.plus(fb.times(s));
        
        var r = v.magnitude();
        var l = w.magnitude();
        // If w is shorter than v,
        if (l <= r) {
            // Calculate how far we need to move along the line to be the
            // same distance from the center as v.
            var z = Math.sqrt(r*r - l*l);
            console.log("z " + z);
            // Move along the line that amount, in the same direction as v.
            if (v.dot(fb) > 0)
                w = w.plus(fb.times(z));
            else
                w = w.minus(fb.times(z));
        }
        else {
            // Otherwise scale w down to the magnitude of v.
            w.times(r/l);
        }
        
        // v is the x of a coordinate system.
        var vx = v;
        vx = vx.times(1/vx.magnitude());
        
        // w is the x of another coordinate system
        var wx = w;
        wx = wx.times(1/wx.magnitude());
        
        // Both systems should use v x w as their z direction.
        // So they are rotation about this z axis.
        var vz = vx.cross(wx);
        // Too short -- danger of divide by zero.
        if (vz.magnitude() < 1e-3)
            return;
        vz = vz.times(1/vz.magnitude());
        var wz = vz;
        
        // If you have x and z, you can get y.
        var vy = vz.cross(vx);
        var wy = wz.cross(wx);
        // Transforms [1 0 0 0]^T to vx
        var vMat = new Mat(vx, vy, vz);
        // Transforms [1 0 0 0]^T to wx
        var wMat = new Mat(wx, wy, wz);
        // Tranforms vx to [1 0 0 0]^T to wx.
        var vwMat = wMat.times(vMat.transpose());
        
        
        //EXERCISE 9B
        // Update orientation using vwMat.
        this.part2joint = vwMat.times(this.part2joint);
        this.joint2part = this.joint2part.times(vwMat.transpose());


    }
}


function makeLeftUpperArm (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 4, 12, new PV(.7,.7,.7));
    var t = new PV(0.707, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.80, 0.25, 0.2, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.80, 4.0, 5.0, false)));
    var upperArm = new Part(sphere, 3,
                            sphere2part, part2sphere,
                            joint2parent, parent2joint,0);
    {
        var t = new PV(1.3, 0, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,-2.0)).times(Mat.rotation(1,-.2));
        var parent2child = Mat.rotation(1,.2).times(Mat.rotation(2,2.0)).times(Mat.translation(t.minus()));
        upperArm.children.push(makeLowerArm(gl, child2parent, parent2child));
    }
    return upperArm;
}

function makeRightUpperArm (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 4, 12, new PV(.7,.7,.7));
    var t = new PV(0.707, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.80, 0.25, 0.2, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.80, 4.0, 5.0, false)));
    var upperArm = new Part(sphere, 3,
                            sphere2part, part2sphere,
                            joint2parent, parent2joint, 0);
    {
        var t = new PV(1.2, 0, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(1,.5)).times(Mat.rotation(2,2.4));
        var parent2child = Mat.rotation(2,-2.4).times(Mat.rotation(1,-.5)).times(Mat.translation(t.minus()));
        upperArm.children.push(makeRightLowerArm(gl, child2parent, parent2child));
    }
    return upperArm;
}

function makeLowerArm (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 4, 12, new PV(.7,.7,.7));
    var t = new PV(.9, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.80, 0.2, 0.2, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.80, 5.0, 5.0, false)));
    var lowerArm = new Part(sphere, 3,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 0);
    {
        var t = new PV(1.4, 0, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,-.5));
        var parent2child = Mat.rotation(2,.5).times(Mat.translation(t.minus()));
        lowerArm.children.push(makeHand(gl, child2parent, parent2child));
    }
   
    return lowerArm;
}
function makeRightLowerArm (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 4, 12, new PV(.7, .7, .7));
    var t = new PV(0.80, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.80, 0.2, 0.2, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.80, 5.0, 5.0, false)));
    var lowerArm = new Part(sphere, 3,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 0);
    {
        var t = new PV(1.4, 0, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,-.5)).times(Mat.rotation(0,-.5));
        var parent2child = Mat.rotation(0,.5).times(Mat.rotation(2,.5).times(Mat.translation(t.minus())));
        lowerArm.children.push(makeHand(gl, child2parent, parent2child));
    }
   
    return lowerArm;
}
function makeHand (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 4, 16, new PV(1.5,1.5,1.5));
    var t = new PV(0.7, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.3, .15, .15, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.30, 1/.15, 1/.15, false)));
    var hand = new Part(sphere, 3,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 0);
    {
        var t = new PV(.4, .05, 0, false);
        var child2parent = Mat.translation(t);
        var parent2child = Mat.translation(t.minus());
        hand.children.push(makeFinger(gl, child2parent, parent2child));
    }
    {
        var t = new PV(.4, -.05, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,-.5));
        var parent2child = Mat.rotation(2,.5).times(Mat.translation(t.minus()));
        hand.children.push(makeFinger(gl, child2parent, parent2child));
    }
    {
        var t = new PV(.4, 0, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,-.25));
        var parent2child = Mat.rotation(2,.25).times(Mat.translation(t.minus()));
        hand.children.push(makeFinger(gl, child2parent, parent2child));
    }
   
    return hand;
}
function makeFinger (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 16, 12, new PV(1.5,1.5,1.5));
    var t = new PV(0.80, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.25, 0.05, 0.05, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.25, 1/.05, 1/.05, false)));
    return new Part(sphere, 3,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 0);


}

function makeRightUpperLeg (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 12, 2, new PV(.7,.7,.7));
    var t = new PV(0.707, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.80, 0.3, 0.3, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.80, 1/.3, 1/.3, false)));
    var upperLeg = new Part(sphere, 3,
                            sphere2part, part2sphere,
                            joint2parent, parent2joint);
    {
        var t = new PV(1.1, 0, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,.4));
        var parent2child = Mat.rotation(2,-.4).times(Mat.translation(t.minus()));
        upperLeg.children.push(makeLowerLeg(gl, child2parent, parent2child));
    }
    return upperLeg;
}
function makeLeftUpperLeg (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 12, 2, new PV(.7,.7,.7));
    var t = new PV(0.707, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.80, 0.3, 0.3, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.80, 1/.3, 1/.3, false)));
    var upperLeg = new Part(sphere, 3,
                            sphere2part, part2sphere,
                            joint2parent, parent2joint, 0);
    {
        var t = new PV(1.10, 0, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,-.1));
        var parent2child = Mat.rotation(2,.1).times(Mat.translation(t.minus()));
        upperLeg.children.push(makeLowerLeg(gl, child2parent, parent2child));
    }
    return upperLeg;
}

function makeLowerLeg (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 12, 2, new PV(.7,.7,.7));
    var t = new PV(0.750, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.6, 0.3, 0.3, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.6, 1/.3, 1/.3, false)));
    var lowerLeg = new Part(sphere, 3,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 0);
    {
        var t = new PV(0.9, 0, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(1,-.25));
        var parent2child = Mat.rotation(1,.25).times(Mat.translation(t.minus()));
        lowerLeg.children.push(makeShoe(gl, child2parent, parent2child));
    }
    return lowerLeg;
}

function makeShoe (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 16, 2, new PV(1.5,1.5,1.5));
    var t = new PV(0, 0, .6, false);
    var sphere2part = Mat.scale(new PV(.15, 0.4, 0.65, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.15, 1/.4, 1/.65, false)));
    return new Part(sphere, 3,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 0);

}

function makeLeftEye (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 16, 16, new PV(.25,.9,.3));
    var t = new PV(0, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.3, 0.3, 0.1, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.3, 1/.3, 1/.1, false)));
    var eye = new Part(sphere, 4,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 1);
    {
        var t = new PV(0, -.01, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(0,-.2));
        var parent2child = Mat.rotation(0,.2).times(Mat.translation(t.minus()));
        eye.children.push(makeEyeIn(gl, child2parent, parent2child));
    }
    {
        var t = new PV(0, -.01, 0, false);
        var child2parent = Mat.translation(t);
        var parent2child = Mat.translation(t.minus());
        eye.children.push(makeLeftEyeLash(gl, child2parent, parent2child));
    }
    return eye;
}
function makeRightEye (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 16, 16, new PV(.25,.9,.3));
    var t = new PV(0, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.3, 0.3, 0.1, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.3, 1/.3, 1/.1, false)));
    var eye = new Part(sphere, 4,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 1);
    {
        var t = new PV(0, -.01, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(0,-.2));
        var parent2child = Mat.rotation(0,.2).times(Mat.translation(t.minus()));
        eye.children.push(makeEyeIn(gl, child2parent, parent2child));
    }
    {
        var t = new PV(0, -.01, 0, false);
        var child2parent = Mat.translation(t);
        var parent2child = Mat.translation(t.minus());
        eye.children.push(makeRightEyeLash(gl, child2parent, parent2child));
    }
    return eye;
}

function makeLeftEyeLash (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 8, 8, new PV(.05,.05,.05));
    var t = new PV(0, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.3, 0.02, 0.15, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.3, 1/.02, 1/.15, false)));
    var eyeLash = new Part(sphere, 3,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 1);
    {
        var t = new PV(.20, -.04, .05, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,-.4));
        var parent2child = Mat.rotation(2,.4).times(Mat.translation(t.minus()));
        eyeLash.children.push(makeLash(gl, child2parent, parent2child));
    }
    {
        var t = new PV(.24, 0, .05, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,-.9));
        var parent2child = Mat.rotation(2,.9).times(Mat.translation(t.minus()));
        eyeLash.children.push(makeLash(gl, child2parent, parent2child));
    }

    {   
        var t = new PV(.28, 0, .05, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,-1.4));
        var parent2child = Mat.rotation(2,1.4).times(Mat.translation(t.minus()));
        eyeLash.children.push(makeLash(gl, child2parent, parent2child));
    }



    return eyeLash;
}
function makeRightEyeLash (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 8, 8, new PV(.05,.05,.05));
    var t = new PV(0, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.3, 0.02, 0.15, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.3, 1/.02, 1/.15, false)));
    var eyeLash = new Part(sphere, 3,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 1);
     {
        var t = new PV(-0.20, -.04, .05, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,.4));
        var parent2child = Mat.rotation(2,-.4).times(Mat.translation(t.minus()));
        eyeLash.children.push(makeLash(gl, child2parent, parent2child));
    }
    {
        var t = new PV(-.24, 0, .05, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,.9));
        var parent2child = Mat.rotation(2,-.9).times(Mat.translation(t.minus()));
        eyeLash.children.push(makeLash(gl, child2parent, parent2child));
    }

    {
        var t = new PV(-.28, 0, .05, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,1.4));
        var parent2child = Mat.rotation(2,-1.4).times(Mat.translation(t.minus()));
        eyeLash.children.push(makeLash(gl, child2parent, parent2child));
    }



    return eyeLash;
}
function makeLash (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 4, 8, new PV(.05,.05,.05));
    var t = new PV(0, .9, 0, false);
    var sphere2part = Mat.scale(new PV(.009, 0.1, 0.05, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.009, 1/.1, 1/.05, false)));
    var lash = new Part(sphere, 3,
                    sphere2part, part2sphere,
                    joint2parent, parent2joint, 1);

    return lash;
}



function makeEyeIn (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 8, 8, new PV(1.5,1.5,1.5));
    var t = new PV(0, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.3, 0.3, 0.1, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.3, 1/.3, 1/.1, false)));
    var eyeIn = new Part(sphere, 4, sphere2part, part2sphere, joint2parent, parent2joint, 1);
    {
        var t = new PV(.075, -.12, .025, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(0,.3));
        var parent2child = Mat.rotation(0,.3).times(Mat.translation(t.minus()));
        eyeIn.children.push(makePupil(gl, child2parent, parent2child));
    }

    return eyeIn;
}
function makePupil (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 12, 12, new PV(.05,.05,.05));
    var t = new PV(0, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.075, 0.15, 0.1, false)).times(Mat.translation(t));
    var part2sphere = Mat.translation(t.minus()).times(Mat.scale(new PV(1/.075, 1/.15, 1/.1, false)));
    var pupil = new Part(sphere, 4, sphere2part, part2sphere, joint2parent, parent2joint, 1);
    {
        var t = new PV(.025, 0, .04, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(0,.3));
        var parent2child = Mat.rotation(0,-.3).times(Mat.translation(t.minus()));
        pupil.children.push(makePupilIn(gl, child2parent, parent2child));
    }
    return pupil;
}
function makePupilIn (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 12, 12, new PV(1.5,1.5,1.5));
    var t = new PV(0, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.0375, 0.075, 0.08, false));
    var part2sphere = Mat.scale(new PV(1/.0375, 1/.075, 1/.08, false));
    return pupil = new Part(sphere, 4, sphere2part, part2sphere, joint2parent, parent2joint, 1);
}

function makeBrow (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 4, 8, new PV(.05,.05,.05));
    var t = new PV(0, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.2, 0.04, 0.03, false));
    var part2sphere = Mat.scale(new PV(1/.04, 1/.04, 1/.01, false));
    var brow = new Part(sphere, 3, sphere2part, part2sphere, joint2parent, parent2joint, 1);
    {
        var t = new PV(.28, -.05, 0, false);
        var child2parent = Mat.translation(t).times(Mat.rotation(2,-.5));
        var parent2child = Mat.rotation(0,-.3).times(Mat.translation(t.minus()));
        brow.children.push(makeOutBrow(gl, child2parent, parent2child));
    }
    return brow;

}
function makeOutBrow (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 4, 8, new PV(0.05,.05,.05));
    var t = new PV(.4, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.15, 0.025, 0.01, false));
    var part2sphere = Mat.scale(new PV(1/.0375, 1/.075, 1/.05, false));
    return (new Part(sphere, 3, sphere2part, part2sphere, joint2parent, parent2joint, 1));
}

function makeLip (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 8, 9, new PV(.1,.41,.12));
    var t = new PV(0, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.7, 0.05, 0.1, false));
    var part2sphere = Mat.scale(new PV(1/.7, 1/.05, 1/.1, false));
    var lip = new Part(sphere, 4, sphere2part, part2sphere, joint2parent, parent2joint, 1);
    return lip;
}
function makeUpperLip (gl, joint2parent, parent2joint) {
    var sphere = new Sphere(gl, 8, 9, new PV(.1,.41,.12));
    var t = new PV(0, 0, 0, false);
    var sphere2part = Mat.scale(new PV(.42, 0.07, 0.05, false));
    var part2sphere = Mat.scale(new PV(1/.42, 1/.07, 1/.2, false));
    var lip = new Part(sphere, 4, sphere2part, part2sphere, joint2parent, parent2joint, 1);
    return lip;
}

function makeBody (gl) {
    // EXERCISE 3:
    // Test with simple = false (sphere body no appendages) and true
    // (body with arm, leg, and extra head)
    // Switch back and forth for tests.
    // Remove the extra head and add the other arm and leg.
    // Put some antennae on the head.
    var simple = false;
    var body = null;
    if (simple) {
        var sphere = new Sphere(gl, 32, 16, new PV(1,0,0, true));
        var id = new Mat();
        body = new Part(sphere, 2, id, id, id, id);
    }
    else {
        var sphere = new Sphere(gl, 32, 32, new PV(.20,.82,.24));
        var sphere2part = Mat.scale(new PV(2, 2, 1/1.5, true))
            .times(Mat.rotation(2, .05 /*Math.PI/4*/));
        var part2sphere = Mat.rotation(2, -.05 /*-Math.PI/4*/)
            .times(Mat.scale(new PV(1.0/2, 1/2, 1.5, true)));
        
        body = new Part(sphere, 2, sphere2part, part2sphere,
                        new Mat(), new Mat(), 0);
        

        {
            var t = new PV(0, .15,.7, false);
            var child2body = Mat.translation(t).times(Mat.rotation(0,1));
            var body2child = Mat.rotation(0,-1).times(Mat.translation(t.minus()));
            body.children.push(makeLip(gl, child2body, body2child));
        }
            {
            var t = new PV(.32, .25, .7, false);
            var child2parent = Mat.translation(t).times(Mat.rotation(2,-.2));
            var parent2child = Mat.rotation(2,.2).times(Mat.translation(t.minus()));
            body.children.push(makeUpperLip(gl, child2parent, parent2child));
            }
            {
            var t = new PV(-.32, .25, .7, false);
            var child2parent = Mat.translation(t).times(Mat.rotation(2,.2));
            var parent2child = Mat.rotation(2,-.2).times(Mat.translation(t.minus()));
            body.children.push(makeUpperLip(gl, child2parent, parent2child));
            }

        {
            var t = new PV(.4, .8,.6, false);
            var child2body = Mat.translation(t).times(Mat.rotation(0,0)).times(Mat.rotation(1,.15));
            var body2child = Mat.rotation(1,-.15).times(Mat.translation(t.minus()));
            body.children.push(makeLeftEye(gl, child2body, body2child));
        }
        {
            var t = new PV(-.325, .8,.6, false);
            var child2body = Mat.translation(t).times(Mat.rotation(0,0)).times(Mat.rotation(1,-.15));
            var body2child = Mat.rotation(1,.15).times(Mat.translation(t.minus()));
            body.children.push(makeRightEye(gl, child2body, body2child));
        }
        {
            var t = new PV(.25, 1.25,.6, false);
            var child2body = Mat.translation(t).times(Mat.rotation(2,.1));
            var body2child = Mat.rotation(2,-.1).times(Mat.translation(t.minus()));
            body.children.push(makeBrow(gl, child2body, body2child));
        }
        {
            var t = new PV(-.25, 1.25,.6, false);
            var child2body = Mat.translation(t).times(Mat.rotation(1,3.1415)).times(Mat.rotation(2,.1));
            var body2child = Mat.rotation(2,-.1).times(Mat.rotation(1,-3.1415).times(Mat.translation(t.minus())));
            body.children.push(makeBrow(gl, child2body, body2child));
        }

    
        {
            var t = new PV(1.75, .25, .1, false);
            var child2body = Mat.translation(t).times(Mat.rotation(2,-.4));
            var body2child = Mat.rotation(2,.4).times(Mat.translation(t.minus()));
            body.children.push(makeLeftUpperArm(gl, child2body, body2child));
        }
        {
            var t = new PV(-1.75, .25, .1, false);
            var child2body = Mat.translation(t).times(Mat.rotation(0, -3.1415)).times(Mat.rotation(1, -3.1415)).times(Mat.rotation(2,.9));
            var body2child = Mat.rotation(2,-.9).times(Mat.rotation(1, 3.1415)).times(Mat.rotation(0, 3.1415)).times(Mat.translation(t.minus()));
            body.children.push(makeRightUpperArm(gl, child2body, body2child));
        }
        
        {
            var t = new PV(0.5, -2.8 * 0.707, 0, false);
            var child2body = Mat.translation(t).times(Mat.rotation(2, -3.1415/2 +.15));
            var body2child = Mat.rotation(2, 3.1415/2 -.15).times(Mat.translation(t.minus()));
            body.children.push(makeLeftUpperLeg(gl, child2body, body2child));
        }
        {
            var t = new PV(-0.6, -2.8 * 0.707, 0, false);
            var child2body = Mat.translation(t).times(Mat.rotation(2, -3.1415/2 -.2));
            var body2child = Mat.rotation(2, 3.1415/2 +.2).times(Mat.translation(t.minus()));
            body.children.push(makeRightUpperLeg(gl, child2body, body2child));
        }
           }
    
return body;
}
function makeMirror (gl) {
    var mirror = null;
    var sphere = new Sphere(gl, 4, 2, new PV(0,0,0));
    var t = new PV (8,0,-2);
        var sphere2part = Mat.scale(new PV(4, 4, .1, true));
        var part2sphere = Mat.scale(new PV(1.0/.25, 1/4, 1/4, true));
        mirror = new Part(sphere, 0, sphere2part, part2sphere,
                        Mat.translation(t), Mat.translation(t.minus()));
        
    return mirror;
}


        

    
