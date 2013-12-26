Math.sign = function (num) {
	if (num > 0) {
		return 1;
	}
	if (num < 0) {
		return -1;
	}
	return 0;
};

// from http://www.webmasterworld.com/javascript/3551991.htm
function pointInPolygon(p, x, y) {
	var npol = p.length, i, j, c = 0;
	for (i = 0, j = npol - 1; i < npol; j = i++) {
		if ((((p[i].y <= y) && (y < p[j].y)) ||
			((p[j].y <= y) && (y < p[i].y))) &&
			(x < (p[j].x - p[i].x) * (y - p[i].y) / (p[j].y - p[i].y) + p[i].x)) {
			c = !c;
		}
	}
	return c;
}

/*
 * Now for some actual objects
 */

function Point(x, y) {
	var self = this;
	this.x = x;
	this.y = y;
}

function Button(description, gcode) {
	var self = this;
	this.description = description;
	this.gcode = gcode;

	this.points = [];
}

Button.prototype = {
	check: function(x, y) {
		return pointInPolygon(this.points, x, y);
	},
	toString: function() {
		return this.description;
	},
	highlight: function(context) {
		var i;
		context.save();
		context.lineWidth = 2;
		context.strokeStyle = "rgba(255, 255, 64, 1)";
		context.fillStyle = "rgba(255, 255, 64, 0.25)";
		context.beginPath();
		context.moveTo(this.points[this.points.length - 1].x, this.points[this.points.length - 1].y);
		for (i = 0; i < this.points.length; i++) {
			context.lineTo(this.points[i].x, this.points[i].y);
		}
		context.stroke();
		context.fill();
		context.restore();
	}
};

function Jog(canvas) {
	var self = this;
	this.eventListeners = {
		click: []
	};
	this.fireEvent = function(name, e) {
		if (self.eventListeners[name]) {
			if (self.eventListeners[name].length > 0) {
				var a, f;
				for (f = 0; f < self.eventListeners[name].length; f++) {
					self.eventListeners[name][f](e);
				}
			}
		}
	};
	
	this.buttons = [];
	this.canvas = canvas;

	if (!canvas) {
		return this;
	}
	
	this.context = canvas.getContext('2d');
	this.context.circle = function(x, y, radius) {
		this.save();
		this.beginPath();
		this.arc(x, y, radius, 0, Math.PI * 2, true);
		this.stroke();
		this.restore();
	};
	this.context.line = function(x1, y1, x2, y2) {
		this.save();
		this.beginPath();
		this.moveTo(x1, y1);
		this.lineTo(x2, y2);
		this.stroke();
		this.restore();
	};
	
	this.canvas.onmousemove = function(e) {
		var x = e.clientX - $(self.canvas).viewportOffset().left;
		var y = e.clientY - $(self.canvas).viewportOffset().top;
		var b;
		self.draw();
		for (b = 0; b < self.buttons.length; b++) {
			if (self.buttons[b].check(x, y)) {
				// TODO: highlight
				self.buttons[b].highlight(self.context);
			}
		}
	};
	this.canvas.onclick = function(e) {
		var x = e.clientX - $(self.canvas).viewportOffset().left;
		var y = e.clientY - $(self.canvas).viewportOffset().top;
		var b;
		for (b = 0; b < self.buttons.length; b++) {
			var button = self.buttons[b];
			if (button.check(x, y)) {
				e.description = button.description;
				e.gcode = button.gcode;
				self.fireEvent('click', e);
			}
		}
	};
	this.canvas.onmouseout = function(e) {
		self.draw();
	};
}

Jog.prototype = {
	draw: function() {
		var canvas = this.canvas;
		var context = this.context;
		context.clearRect(0, 0, canvas.width, canvas.height);
	},
	observe: function(event, callback) {
		if (this.eventListeners[event]) {
			this.eventListeners[event].push(callback);
		}
	}
};

/*
 * XY jog buttons
 */

ButtonXY.prototype = new Button();
ButtonXY.prototype.constructor = ButtonXY;
function ButtonXY(axis, length, canvas) {
	var self = this, a;

	Button.call(this, axis + length, [ "G91", "G1 " + axis + length, "G90" ]);
	
	var innerR;
	var outerR;
	var centerX = 247 * 0.5;
	var centerY = 242 * 0.5;
	var quadrant;
	if (length == 0.1 || length == -0.1) {
		innerR = 0;
		outerR = 32.3;
	}
	if (length == 1   || length == -1  ) {
		innerR = 32.3;
		outerR = 57.75;
	}
	if (length == 10  || length == -10 ) {
		innerR = 57.76;
		outerR = 82;
	}
	if (length == 100 || length == -100) {
		innerR = 82;
		outerR = 103;
	}
	
	if (axis == 'X') {
		quadrant = 1;
		centerX += 10;
	}
	else if (axis == 'Y') {
		quadrant = 0;
		centerY -= 10;
	}
	if (length < 0) {
		if (quadrant == 1) {
			centerX -= 20;
		}
		else if (quadrant == 0) {
			centerY += 20;
		}
		quadrant += 2;
	}
	
	this.innerR = innerR;
	this.outerR = outerR;
	this.quadrant = quadrant;

	for (a = 45; a <= 135; a += 5) {
		this.points.push(new Point(centerX + innerR * Math.cos((quadrant * 90 - a) * Math.PI / 180), centerY + innerR * Math.sin((quadrant * 90 - a) * Math.PI / 180)));
	}
	for (a = 135; a >= 45; a -= 5) {
		this.points.push(new Point(centerX + outerR * Math.cos((quadrant * 90 - a) * Math.PI / 180), centerY + outerR * Math.sin((quadrant * 90 - a) * Math.PI / 180)));
	}
}

JogXY.prototype = new Jog();
JogXY.prototype.constructor = JogXY;
function JogXY(canvas) {
	var self = this, d;
	
	Jog.call(this, canvas);
	
	for (d = 0.1; d < 101; d *= 10) {
		this.buttons.push(new ButtonXY('X',  d));
		this.buttons.push(new ButtonXY('X', -d));
		this.buttons.push(new ButtonXY('Y',  d));
		this.buttons.push(new ButtonXY('Y', -d));
	}
	var Xhome = new Button("Home X", [ "G28 X0" ]);
	Xhome.points.push(new Point(11     , 8));
	Xhome.points.push(new Point(11 + 47, 8));
	Xhome.points.push(new Point(11 + 47, 8 +  9));
	Xhome.points.push(new Point(11 + 26, 8 + 26));
	Xhome.points.push(new Point(11 +  9, 8 + 47));
	Xhome.points.push(new Point(11     , 8 + 47));
	this.buttons.push(Xhome);
	
	var Yhome = new Button("Home Y", [ "G28 Y0" ]);
	Yhome.points.push(new Point(235     , 8));
	Yhome.points.push(new Point(235 - 47, 8));
	Yhome.points.push(new Point(235 - 47, 8 +  9));
	Yhome.points.push(new Point(235 - 26, 8 + 26));
	Yhome.points.push(new Point(235 -  9, 8 + 47));
	Yhome.points.push(new Point(235     , 8 + 47));
	this.buttons.push(Yhome);

	var Zhome = new Button("Home Z", [ "G28 Z0" ]);
	Zhome.points.push(new Point(236     , 232));
	Zhome.points.push(new Point(236 - 47, 232));
	Zhome.points.push(new Point(236 - 47, 232 -  9));
	Zhome.points.push(new Point(236 - 26, 232 - 26));
	Zhome.points.push(new Point(236 -  9, 232 - 47));
	Zhome.points.push(new Point(236     , 232 - 47));
	this.buttons.push(Zhome);
	
	var Allhome = new Button("Home All", [ "G28 X0 Y0 Z0" ]);
	Allhome.points.push(new Point(11     , 232));
	Allhome.points.push(new Point(11 + 47, 232));
	Allhome.points.push(new Point(11 + 47, 232 -  9));
	Allhome.points.push(new Point(11 + 26, 232 - 26));
	Allhome.points.push(new Point(11 +  9, 232 - 47));
	Allhome.points.push(new Point(11     , 232 - 47));
	this.buttons.push(Allhome);
}

/*
 * Z and E jog buttons
 */

ButtonZE.prototype = new Button();
ButtonZE.prototype.constructor = ButtonZE;
function ButtonZE(axis, length, x1, y1, x2, y2) {
	Button.call(this, axis + length, [ "G91", "G1 " + axis + length, "G90" ]);
	
	this.points.push(new Point(x1, y1));
	this.points.push(new Point(x2, y1));
	this.points.push(new Point(x2, y2));
	this.points.push(new Point(x1, y2));
}

ButtonZ.prototype = new ButtonZE();
ButtonZ.prototype.constructor = ButtonZ;
function ButtonZ(axis, length) {
	var x1 = 12;
	var x2 = 47;
	var y = [200, 172, 148, 126, 111, 90, 65, 38];
	var q = ((Math.round(Math.log(Math.abs(length)) / Math.log(10))) + 2) * Math.sign(length) + 3;

	var y1 = y[q];
	var y2 = y[q + 1];
	
	ButtonZE.call(this, axis, length, x1, y1, x2, y2);
}

JogZ.prototype = new Jog();
JogZ.prototype.constructor = JogZ;
function JogZ(canvas) {
	var self = this, d;

	Jog.call(this, canvas);
	
	this.axis = 'Z';
	
	for (d = 0.1; d < 11; d *= 10) {
		this.buttons.push(new ButtonZ(this.axis, d));
		this.buttons.push(new ButtonZ(this.axis, -d));
	}
}

ButtonE.prototype = new ButtonZE();
function ButtonE(axis, length) {
	var x1 = 12;
	var x2 = 47;
	var q = (((Math.abs(length) == 1)?0:((Math.abs(length) == 5)?1:2)) + 1) * Math.sign(length) + 3;

	var y = [200, 172, 148, 126, 111, 90, 65, 38];
	
	var y1 = y[q];
	var y2 = y[q + 1];
	
	ButtonZE.call(this, axis, length, x1, y1, x2, y2);
}

JogE.prototype = new Jog();
JogE.prototype.constructor = JogE;
function JogE(canvas, axis) {
	var self = this, d;
	
	Jog.call(this, canvas);
	
	this.axis = 'E';
	
	for (d = 0; d < 3; d++) {
		var v = (d == 0)?1:((d == 1)?5:10);
		this.buttons.push(new ButtonE(this.axis, v));
		this.buttons.push(new ButtonE(this.axis, -v));
	}
}
