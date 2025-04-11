		// Display the countdown
		var countdown = document.getElementById("countdown");
		var remainingSeconds = 3;
		countdown.innerHTML = "The Robot will Start moving in " + remainingSeconds + " seconds.";
		var countdownInterval = setInterval(function() {
			remainingSeconds--;
			if (remainingSeconds == 0) {
				clearInterval(countdownInterval);
				countdown.style.display = "none";
				document.getElementById("content").style.display = "block";
			} else {
				countdown.innerHTML = "Count Down: " + remainingSeconds + " seconds.";
			}
		}, 1000);