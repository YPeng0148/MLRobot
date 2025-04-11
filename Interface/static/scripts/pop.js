var radioButtons = document.querySelectorAll('input[type="radio"]');
radioButtons.forEach(function(button) {
    button.addEventListener('change', function() {
        // Remove the yellow highlight from all radio buttons
        radioButtons.forEach(function(button) {
            button.parentNode.classList.remove('selected');
        });
        // Add the yellow highlight to the currently selected radio button
        if (this.checked) {
            this.parentNode.classList.add('selected');
            // Display the selected image as a pop-up
            var imageUrl = document.querySelector('.image-' + this.value).src;
            var popUp = document.createElement('div');
            popUp.classList.add('pop-up');
            popUp.innerHTML = '<img src="' + imageUrl + '">';
            document.body.appendChild(popUp);
        } else {
            // Remove the pop-up if the radio button is unselected
            var popUp = document.querySelector('.pop-up');
            if (popUp) {
                popUp.remove();
            }
        }
    });
});

// Add a click event listener to each image to close the pop-up when clicked
var images = document.querySelectorAll('.container img');
images.forEach(function(image) {
    image.addEventListener('click', function() {
        var popUp = document.querySelector('.pop-up');
        if (popUp) {
            popUp.remove();
        }
    });
});
