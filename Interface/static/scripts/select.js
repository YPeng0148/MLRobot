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
        }
    });
});