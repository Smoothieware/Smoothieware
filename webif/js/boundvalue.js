// constructor for numeric values bound to input fields
function boundValue(inputElement, defaultValue, min, max, integer) {
	var self = this;

	this.input = $(inputElement);
	this.defaultValue = defaultValue;
	this.value = undefined;
	this.normalBackground = this.input.style.backgroundColor;
	this.lastValidValue = defaultValue;
	this.max = max;
	this.min = min;
	this.integer = integer;

	this.input.observe('blur', function(e) {
		try {
			self.set(this.value);
			this.style.backgroundColor = self.normalBackground;
		}
		catch (e) {
			self.normalBackground = this.style.backgroundColor;
			this.style.backgroundColor = 'orange';
			alert(e);
			this.value = self.lastValidValue;
			this.focus();
		}
	});

	this.input.observe('change', function(e) {
		this.style.backgroundColor = self.normalBackground;
	});

	this.set(this.defaultValue);
}

boundValue.prototype = {
	checkValue: function(newvalue) {
		if (!isNaN(newvalue) && isFinite(newvalue)) {
			if (!isNaN(this.max) && newvalue > this.max) {
				newvalue = this.max;
			}
			if (!isNaN(this.min) && newvalue < this.min) {
				newvalue = this.min;
			}
			if (this.integer)
				newvalue = parseInt(newvalue);
			else
				newvalue = parseFloat(newvalue);
			return newvalue;
		}
		else
			throw newvalue + " is not numeric!";
	},
	set: function(newvalue) {
		newvalue = this.checkValue(newvalue);
		this.value = newvalue
		this.input.value = newvalue;
		this.lastValidValue = newvalue;
	},
	setMin: function(newvalue) {
		if (!isNaN(newvalue) && isFinite(newvalue)) {
			if (this.integer)
				newvalue = parseInt(newvalue);
			else
				newvalue = parseFloat(newvalue);
			this.min = newvalue;
			this.set(this.value);
		}
		else
			throw newvalue + " is not numeric!";
	},
	setMax: function(newvalue) {
		if (!isNaN(newvalue) && isFinite(newvalue)) {
			if (this.integer)
				newvalue = parseInt(newvalue);
			else
				newvalue = parseFloat(newvalue);
			this.max = newvalue;
			this.set(this.value);
		}
		else
			throw newvalue + " is not numeric!";
	},
	toString: function() {
		return this.value + "";
	}
};
