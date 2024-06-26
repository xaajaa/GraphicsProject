// o: origin of picture in world
// vx: unit x-axis of picture in world
// vy: unit y-axis of picture in world
// w: width of picture in world
// h: height of picture in world
function Picture (gl, o, vx, vy, w, h) {
    // EXERCISE 1A
    // Set this.picture2world and this.world2picture for this picture.
    // Point (0, 0, 0) in the picture is o in the world.
    // Point (1, 1, 0) in the picture is o + vx w + vy h in the world.
    var vz = vx.cross(vy);
    var R = new Mat(vx,vy,vz, 0);

    this.picture2world = Mat.translation(o).times(R).times(Mat.scale(new PV(w, h, 1, 1)));
    this.world2picture = Mat.scale(new PV(1/w, 1/h, 1,1)).times(R.transpose()).times(Mat.translation(o.minus()));
    
    console.log("picture check\n" + this.picture2world.times(this.world2picture));

    var verts = [ new PV(0, 0, 0, true),
                  new PV(1, 0, 0, true),
                  new PV(1, 1, 0, true),
                  new PV(0, 1, 0, true) ];

    var buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(verts), gl.STATIC_DRAW);

    this.render = function (gl, program) {
        var vPosition = gl.getAttribLocation(program, "vPosition");
        var vNormal = gl.getAttribLocation(program, "vNormal");
        var colorLoc = gl.getUniformLocation(program, "color");
        var normalLoc = gl.getUniformLocation(program, "normal");
        var useNormalLoc = gl.getUniformLocation(program, "useNormal");

        gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
        gl.vertexAttribPointer(vPosition, 4, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(vPosition);

        gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
        gl.vertexAttribPointer(vNormal, 4, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(vNormal);

        var color = new PV(1, 1, 0, 1);
        gl.uniform4fv(colorLoc, color.flatten());
        gl.uniform1i(useNormalLoc, 5);

        gl.drawArrays(gl.TRIANGLE_FAN, 0, 4);
    }
}
