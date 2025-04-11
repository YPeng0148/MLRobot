 // Get all the radio buttons
 const radioButtons = document.querySelectorAll('input[type="radio"]');
          
 // Add an event listener to each radio button
 radioButtons.forEach(radioButton => {
   radioButton.addEventListener('change', event => {
     // Get the parent div of the radio button (i.e. the item)
     const parentDiv = event.target.closest('.item');

     // Remove the "active" class from all items
     document.querySelectorAll('.my-body-class .item').forEach(item => {
       item.classList.remove('active');
     });

     // Add the "active" class to the parent div of the selected radio button
     parentDiv.classList.add('active');
   });
 });