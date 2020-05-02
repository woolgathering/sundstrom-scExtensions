+ SimpleNumber {

	// convert a signed int8 to an unsigned int8
	asUInt8 {
		if (this > 127 or: this < -128) {
			"Value too large (or small): %".format(this).error;
			^this;
		};

		if (this < 0) {
			^256+this;
		} {
			^this;
		};
	}

	asInt8 {
		if (this > 255 or: this < 0) {
			"Value too large (or small): %".format(this).error;
			^this;
		};

		if (this > 127) {
			^this-256;
		} {
			^this;
		};
	}

	// convert a signed int16 to an unsigned int16
	asUInt16 {
		if (this > 16383 or: this < -16384) {
			"Value too large (or small): %".format(this).error;
			^this;
		};

		if (this < 0) {
			^32768+this;
		} {
			^this;
		};
	}

	asInt16 {
		if (this > 32767 or: this < 0) {
			"Value too large (or small): %".format(this).error;
			^this;
		};

		if (this > 16383) {
			^this-32768;
		} {
			^this;
		};
	}

}
