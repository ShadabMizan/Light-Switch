let lastPowerClick = 0;

const powerBtn = document.getElementById('powerBtn');
const powerSlider = document.getElementById('powerSlider');

const evtSource = new EventSource('/events');

// Current line status (0 = OFF, 1 = ON)
let lineStatus = 0;

// Debounced power button with immediate color feedback
powerBtn.addEventListener('click', () => {
    const now = Date.now();
    if (now - lastPowerClick >= 250) {
        // Optimistically toggle color immediately
        lineStatus = lineStatus ? 0 : 1;
        powerBtn.style.color = lineStatus ? 'green' : 'red';

        // Send the toggle request
        fetch('/api/toggle', { method: 'POST' });
        lastPowerClick = now;
    }
});

// Live slider updates
let sliderInterval = null;
powerSlider.addEventListener('input', () => {
    if (!sliderInterval) {
        sliderInterval = setInterval(() => {
            fetch('/api/set_power', { method: 'POST', body: powerSlider.value });
        }, 100); // every 100ms while moving
    }
});
powerSlider.addEventListener('change', () => {
    clearInterval(sliderInterval);
    sliderInterval = null;
});

evtSource.onmessage = function(event) {
    const value = parseFloat(event.data);
    powerSlider.value = value;
}

// On load: get initial values
window.addEventListener('load', () => {
    fetch('/api/get_power')
        .then(r => r.text())
        .then(val => powerSlider.value = parseFloat(val));

    fetch('/api/get_line_status')
        .then(r => r.text())
        .then(status => {
            lineStatus = parseInt(status);
            powerBtn.style.color = lineStatus ? 'green' : 'red';
        });
});
