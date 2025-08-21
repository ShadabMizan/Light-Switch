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

// Live slider updates (0-1 range)
let sliderInterval = null;
powerSlider.addEventListener('input', () => {
    if (!sliderInterval) {
        sliderInterval = setInterval(() => {
            // Send normalized value between 0-1
            const normalized = parseFloat(powerSlider.value);
            fetch('/api/set_power', { method: 'POST', body: normalized });
        }, 100); // every 100ms while moving
    }
});
powerSlider.addEventListener('change', () => {
    clearInterval(sliderInterval);
    sliderInterval = null;
});

// SSE event handler
evtSource.onmessage = function(event) {
    try {
        const data = JSON.parse(event.data);

        // Update power slider (0-1)
        if (data.power_delivered !== undefined) {
            powerSlider.value = parseFloat(data.power_delivered);
        }

        // Update line status button color
        if (data.line_status !== undefined) {
            lineStatus = parseInt(data.line_status);
            powerBtn.style.color = lineStatus ? 'green' : 'red';
        }
    } catch (err) {
        console.error("Failed to parse SSE event:", err);
    }
};

// On load: get initial values
window.addEventListener('load', () => {
    fetch('/api/get_power')
        .then(r => r.text())
        .then(val => {
            const power = parseFloat(val);
            powerSlider.value = power;
        });

    fetch('/api/get_line_status')
        .then(r => r.text())
        .then(status => {
            lineStatus = parseInt(status);
            powerBtn.style.color = lineStatus ? 'green' : 'red';
        });
});
