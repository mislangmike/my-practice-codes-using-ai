function toggleLaw(law) {
    const lawContainer = document.getElementById(law);
    const currentClass = lawContainer.className;

    if (currentClass.includes('hide')) {
        lawContainer.className = 'transparent-container show';
    } else {
        lawContainer.className = 'transparent-container hide';
    }
}

function calculateWeight() {
    const mass = parseFloat(document.getElementById('mass').value);
    const gravity = parseFloat(document.getElementById('gravity').value);
    if (!isNaN(mass) && !isNaN(gravity)) {
        const weight = mass * gravity;
        document.getElementById('result').innerHTML = `Weight: ${weight.toFixed(2)} N`;
    } else {
        document.getElementById('result').innerHTML = 'Please enter valid values for mass and gravity.';
    }
}

function calculateMass() {
    const weight = parseFloat(document.getElementById('weight').value);
    const gravity = parseFloat(document.getElementById('gravity').value);
    if (!isNaN(weight) && !isNaN(gravity) && gravity !== 0) {
        const mass = weight / gravity;
        document.getElementById('result').innerHTML = `Mass: ${mass.toFixed(2)} kg`;
    } else {
        document.getElementById('result').innerHTML = 'Please enter valid values for weight and gravity.';
    }
}

function calculateGravity() {
    const weight = parseFloat(document.getElementById('weight').value);
    const mass = parseFloat(document.getElementById('mass').value);
    if (!isNaN(weight) && !isNaN(mass) && mass !== 0) {
        const gravity = weight / mass;
        document.getElementById('result').innerHTML = `Gravity: ${gravity.toFixed(2)} m/s²`;
    } else {
        document.getElementById('result').innerHTML = 'Please enter valid values for weight and mass.';
    }
}

function calculateForce() {
    const mass2 = parseFloat(document.getElementById('mass2').value);
    const acceleration = parseFloat(document.getElementById('acceleration').value);
    if (!isNaN(mass2) && !isNaN(acceleration)) {
        const force = mass2 * acceleration;
        document.getElementById('result2').innerHTML = `Force: ${force.toFixed(2)} N`;
    } else {
        document.getElementById('result2').innerHTML = 'Please enter valid values for mass and acceleration.';
    }
}

function calculateMassFromForce() {
    const force = parseFloat(document.getElementById('force').value);
    const acceleration = parseFloat(document.getElementById('acceleration').value);
    if (!isNaN(force) && !isNaN(acceleration) && acceleration !== 0) {
        const mass2 = force / acceleration;
        document.getElementById('result2').innerHTML = `Mass: ${mass2.toFixed(2)} kg`;
    } else {
        document.getElementById('result2').innerHTML = 'Please enter valid values for force and acceleration.';
    }
}

function calculateAcceleration() {
    const force = parseFloat(document.getElementById('force').value);
    const mass2 = parseFloat(document.getElementById('mass2').value);
    if (!isNaN(force) && !isNaN(mass2) && mass2 !== 0) {
        const acceleration = force / mass2;
        document.getElementById('result2').innerHTML = `Acceleration: ${acceleration.toFixed(2)} m/s²`;
    } else {
        document.getElementById('result2').innerHTML = 'Please enter valid values for force and mass.';
    }
}

function displayResult(message) {
    document.getElementById('result').innerText = message;
}

function calculateM1() {
    const mass2 = parseFloat(document.getElementById('mass2').value);
    const acceleration2 = parseFloat(document.getElementById('acceleration2').value);
    const acceleration1 = parseFloat(document.getElementById(' acceleration1').value);

    if (!isNaN(mass2) && !isNaN(acceleration2) && !isNaN(acceleration1) && acceleration1 !== 0) {
        const calculatedM1 = -(mass2 + acceleration2) / acceleration1;
        displayResult(`Calculated Mass 1 (m1): ${calculatedM1.toFixed(2)} kg`);
    } else {
        displayResult('Please enter valid values.');
    }
}

function calculateM2() {
    const mass1 = parseFloat(document.getElementById('mass1').value);
    const acceleration1 = parseFloat(document.getElementById('acceleration1').value);
    const acceleration2 = parseFloat(document.getElementById('acceleration2').value);

    if (!isNaN(mass1) && !isNaN(acceleration1) && !isNaN(acceleration2) && acceleration2 !== 0) {
        const calculatedM2 = -(mass1 + acceleration1) / acceleration2;
        displayResult(`Calculated Mass 2 (-m2): ${calculatedM2.toFixed(2)} kg`);
    } else {
        displayResult('Please enter valid values.');
    }
}

function calculateA1() {
    const mass2 = parseFloat(document.getElementById('mass2').value);
    const acceleration2 = parseFloat(document.getElementById('acceleration2').value);
    const mass1 = parseFloat(document.getElementById('mass1').value);

    if (!isNaN(mass2) && !isNaN(acceleration2) && !isNaN(mass1) && mass1 !== 0) {
        const calculatedA1 = -(mass2 + acceleration2) / mass1;
        displayResult(`Calculated Acceleration 1 (a1): ${calculatedA1.toFixed(2)} m/s²`);
    } else {
        displayResult('Please enter valid values.');
    }
}

function calculateA2() {
    const mass1 = parseFloat(document.getElementById('mass1').value);
    const acceleration1 = parseFloat(document.getElementById('acceleration1').value);
    const mass2 = parseFloat(document.getElementById('mass2').value);

    if (!isNaN(mass1) && !isNaN(acceleration1) && !isNaN(mass2) && mass2 !== 0) {
        const calculatedA2 = -(mass1 + acceleration1) / mass2;
        displayResult(`Calculated Acceleration 2 (-a2): ${calculatedA2.toFixed(2)} m/s²`);
    } else {
        displayResult('Please enter valid values.');
    }
}

// ... rest of your code
function toggleWork() {
    const workContainer = document.getElementById('workContainer');
    const currentClass = workContainer.className;

    if (currentClass.includes('hide')) {
        workContainer.className = 'transparent-container show';
    } else {
        workContainer.className = 'transparent-container hide';
    }
}

function calculateWork() {
    const forceWork = parseFloat(document.getElementById('forceWork').value);
    const distance = parseFloat(document.getElementById('distance').value);
    const angle = parseFloat(document.getElementById('angle').value);

    if (!isNaN(forceWork) && !isNaN(distance)) {
        // Convert angle from degrees to radians
        const angleInRadians = angle * (Math.PI / 180);
        // Calculate work using the formula W = F * d * cos(θ)
        const work = forceWork * distance * Math.cos(angleInRadians);
        document.getElementById('workResult').innerHTML = `Work: ${work.toFixed(2)} J`;
    } else {
        document.getElementById('workResult').innerHTML = 'Please enter valid values for force and distance.';
    }
}

function calculateForceWork() {
    const work = parseFloat(document.getElementById('work').value);
    const distance = parseFloat(document.getElementById('distance').value);
    const forceWork = work / distance;
    document.getElementById('workResult').innerHTML = `Force: ${forceWork.toFixed(2)} N`;
}

function calculateDistance() {
    const work = parseFloat(document.getElementById('work').value);
    const forceWork = parseFloat(document.getElementById('forceWork').value);
    const distance = work / forceWork;
    document.getElementById('workResult').innerHTML = `Distance: ${distance.toFixed(2)} m`;
}

function toggleEnergy() {
    const energyContainer = document.getElementById('energyContainer');
    const currentClass = energyContainer.className;

    if (currentClass.includes('hide')) {
        energyContainer.className = 'transparent-container show';
    } else {
        energyContainer.className = 'transparent-container hide';
    }
}

function calculateKineticEnergy() {
    const massEnergy = document.getElementById('massEnergy').value;
    const velocityEnergy = document.getElementById('velocityEnergy').value;
    const kineticEnergy = 0.5 * massEnergy * Math.pow(velocityEnergy, 2);
    document.getElementById('energyResult').innerHTML = `Kinetic Energy: ${kineticEnergy} J`;
}

function calculatePotentialEnergy() {
    const massEnergy = document.getElementById('massEnergy').value;
    const heightEnergy = document.getElementById('heightEnergy').value;
    const gravityEnergy = document.getElementById('gravityEnergy').value;
    const potentialEnergy = massEnergy * gravityEnergy * heightEnergy;
    document.getElementById('energyResult').innerHTML = `Potential Energy: ${potentialEnergy} J`;
}

function calculateMassFromEnergy() {
    const kineticEnergy = document.getElementById('kineticEnergy').value;
    const velocityEnergy = document.getElementById('velocityEnergy').value;
    const massEnergy = 2 * kineticEnergy / Math.pow(velocityEnergy, 2);
    document.getElementById('energyResult').innerHTML = `Mass: ${massEnergy} kg`;
}

function calculateVelocityFromEnergy() {
    const kineticEnergy = document.getElementById('kineticEnergy').value;
    const massEnergy = document.getElementById('massEnergy').value;
    const velocityEnergy = Math.sqrt(2 * kineticEnergy / massEnergy);
    document.getElementById('energyResult').innerHTML = `Velocity: ${velocityEnergy} m/s`;
}

function calculateHeightFromEnergy() {
    const potentialEnergy = document.getElementById('potentialEnergy').value;
    const massEnergy = document.getElementById('massEnergy').value;
    const gravityEnergy = document.getElementById('gravityEnergy').value;
    const heightEnergy = potentialEnergy / (massEnergy * gravityEnergy);
    document.getElementById('energyResult').innerHTML = `Height: ${heightEnergy} m`;
}

function togglePower() {
    const powerContainer = document.getElementById('powerContainer');
    const currentClass = powerContainer.className;

    if (currentClass.includes('hide')) {
        powerContainer.className = 'transparent-container show';
    } else {
        powerContainer.className = 'transparent-container hide';
    }
}

function calculatePower() {
    const forcePower = document.getElementById('forcePower').value;
    const velocityPower = document.getElementById('velocityPower').value;
    const power = forcePower * velocityPower;
    document.getElementById('powerResult').innerHTML = `Power: ${power} W`;
}

function calculateForcePower() {
    const power = document.getElementById('power').value;
    const velocityPower = document.getElementById('velocityPower').value;
    const forcePower = power / velocityPower;
    document.getElementById('powerResult').innerHTML = `Force: ${forcePower} N`;
}

function calculateVelocityPower() {
    const power = document.getElementById('power').value;
    const forcePower = document.getElementById('forcePower').value;
    const velocityPower = power / forcePower;
    document.getElementById('powerResult').innerHTML = `Velocity: ${velocityPower} m/s`;
}

function convertPower() {
    const powerUnit = document.getElementById('powerUnit').value;
    const powerValue = document.getElementById('powerValue').value;

    let convertedPower;

    switch (powerUnit) {
        case 'W':
            convertedPower = powerValue;
            break;
        case 'kW':
            convertedPower = powerValue * 1000;
            break;
        case 'hp':
            convertedPower = powerValue * 745.7;
            break;
        case 'MW':
            convertedPower = powerValue * 1000000;
            break;
        default:
            convertedPower = 'Invalid unit';
    }

    document.getElementById('powerConversionResult').innerHTML = `Converted Power: ${convertedPower} W`;
}
function toggleImpulse() {
    const impulseContainer = document.getElementById('impulseContainer');
    const currentClass = impulseContainer.className;

    if (currentClass.includes('hide')) {
        impulseContainer.className = 'transparent-container show';
    } else {
        impulseContainer.className = 'transparent-container hide';
    }
}

function calculateImpulse() {
    const forceImpulse = document.getElementById('forceImpulse').value;
    const timeImpulse = document.getElementById('timeImpulse').value;
    const impulse = forceImpulse * timeImpulse;
    document.getElementById('impulseResult').innerHTML = `Impulse: ${impulse} Ns`;
}

function calculateForceImpulse() {
    const impulse = document.getElementById('impulse').value;
    const timeImpulse = document.getElementById('timeImpulse').value;
    const forceImpulse = impulse / timeImpulse;
    document.getElementById('impulseResult').innerHTML = `Force: ${forceImpulse} N`;
}

function calculateTimeImpulse() {
    const impulse = document.getElementById('impulse').value;
    const forceImpulse = document.getElementById('forceImpulse').value;
    const timeImpulse = impulse / forceImpulse;
    document.getElementById('impulseResult').innerHTML = `Time: ${timeImpulse} s`;
}

function toggleMomentum() {
    const momentumContainer = document.getElementById('momentumContainer');
    const currentClass = momentumContainer.className;

    if (currentClass.includes('hide')) {
        momentumContainer.className = 'transparent-container show';
    } else {
        momentumContainer.className = 'transparent-container hide';
    }
}

function calculateMomentum() {
    const massMomentum = document.getElementById('massMomentum').value;
    const velocityMomentum = document.getElementById('velocityMomentum').value;
    const momentum = massMomentum * velocityMomentum;
    document.getElementById('momentumResult').innerHTML = `Momentum: ${momentum} kg·m/s`;
}
function toggleKinematics(equation) {
    const kinematicContainer = document.getElementById(equation);
    const currentClass = kinematicContainer.className;

    // Hide all kinematic containers first
    const containers = document.querySelectorAll('.transparent-container');
    containers.forEach(container => {
        container.className = 'transparent-container hide';
    });

    // Show the selected kinematic container
    if (currentClass.includes('hide')) {
        kinematicContainer.className = 'transparent-container show';
    }
}

function calculateFirstKinematic(type) {
    const u = parseFloat(document.getElementById('u1').value);
    const a = parseFloat(document.getElementById('a1').value);
    const t = parseFloat(document.getElementById('t1').value);
    const vInput = parseFloat(document.getElementById('v1').value);

    if (type === 'v') {
        const v = u + a * t;
        document.getElementById('v1').value = v.toFixed(2);
        document.getElementById('kinematicResult1').innerText = `Final Velocity (v): ${v.toFixed(2)} m/s`;
    } else if (type === 'a') {
        if (t !== 0) {
            const aCalc = (vInput - u) / t;
            document.getElementById('a1').value = aCalc.toFixed(2);
            document.getElementById('kinematicResult1').innerText = `Acceleration (a): ${aCalc.toFixed(2)} m/s²`;
        } else {
            document.getElementById('kinematicResult1').innerText = 'Time cannot be zero for acceleration calculation.';
        }
    } else if (type === 't') {
        if (a !== 0) {
            const tCalc = (vInput - u) / a;
            document.getElementById('t1').value = tCalc.toFixed(2);
            document.getElementById('kinematicResult1').innerText = `Time (t): ${tCalc.toFixed(2)} s`;
        } else {
            document.getElementById('kinematicResult1').innerText = 'Acceleration cannot be zero for time calculation.';
        }
    }
}

function calculateSecondKinematic(variable) {
    const u = parseFloat(document.getElementById('u2').value);
    const t = parseFloat(document.getElementById('t2').value);
    const a = parseFloat(document.getElementById('a2').value);
    const sInput = parseFloat(document.getElementById('s2').value);

    if (variable === 's') {
        const displacement = (u * t) + (0.5 * a * Math.pow(t, 2));
        document.getElementById('kinematicResult2').innerText = `Displacement (s): ${displacement.toFixed(2)} m`;
    } else if (variable === 'a') {
        if (!isNaN(sInput) && !isNaN(u) && !isNaN(t) && t !== 0) {
            const acceleration = (2 * (sInput - (u * t))) / Math.pow(t, 2);
            document.getElementById('kinematicResult2').innerText = `Acceleration (a): ${acceleration.toFixed(2)} m/s²`;
        } else {
            document.getElementById('kinematicResult2').innerText = 'Please enter valid values for displacement, initial velocity, and time to calculate acceleration.';
        }
    } else if (variable === 't') {
        if (!isNaN(sInput) && !isNaN(u) && !isNaN(a) && a !== 0) {
            const discriminant = Math.pow(u, 2) + 2 * a * sInput;
            if (discriminant >= 0) {
                const t1 = (Math.sqrt(discriminant) - u) / a;
                const t2 = (-Math.sqrt(discriminant) - u) / a;
                document.getElementById('kinematicResult2').innerText = `Possible Times (t): ${t1.toFixed(2)} s, ${t2.toFixed(2)} s`;
            } else {
                document.getElementById('kinematicResult2').innerText = 'No real solution for time with the given inputs.';
            }
        } else {
            document.getElementById('kinematicResult2').innerText = 'Please enter valid values for displacement, initial velocity, and acceleration to calculate time.';
        }
    }
}

function calculateThirdKinematic(type) {
    const u = parseFloat(document.getElementById('u3').value);
    const a = parseFloat(document.getElementById('a3').value);
    const s = parseFloat(document.getElementById('s3').value);
    const vInput = parseFloat(document.getElementById('v3').value);

    if (type === 'v') {
        const v = Math.sqrt(u ** 2 + 2 * a * s);
        document.getElementById('v3').value = v.toFixed(2);
        document.getElementById('kinematicResult3').innerText = `Final Velocity (v): ${v.toFixed(2)} m/s`;
    } else if (type === 'a') {
        if (!isNaN(vInput) && !isNaN(u) && !isNaN(s) && s !== 0) {
            const aCalc = (vInput ** 2 - u ** 2) / (2 * s);
            document.getElementById('a3').value = aCalc.toFixed(2);
            document.getElementById('kinematicResult3').innerText = `Acceleration (a): ${aCalc.toFixed(2)} m/s²`;
        } else {
            document.getElementById('kinematicResult3').innerText = 'Please enter valid values for final velocity, initial velocity, and displacement to calculate acceleration.';
        }
    }
}

function calculateFourthKinematic(variable) {
    const v = parseFloat(document.getElementById('v4').value);
    const a = parseFloat(document.getElementById('a4').value);
    const t = parseFloat(document.getElementById('t4').value);
    const sInput = parseFloat(document.getElementById('s4').value);

    if (variable === 's') {
        const displacement = (v * t) - (0.5 * a * Math.pow(t, 2));
        document.getElementById('kinematicResult4').innerText = `Displacement (s): ${displacement.toFixed(2)} m`;
    } else if (variable === 't') {
        if (!isNaN(sInput) && !isNaN(v) && !isNaN(a) && a !== 0) {
            const A = -0.5 * a;
            const B = v;
            const C = -sInput;

            const discriminant = Math.pow(B, 2) - (4 * A * C);
            if (discriminant >= 0) {
                const t1 = (-B + Math.sqrt(discriminant)) / (2 * A);
                const t2 = (-B - Math.sqrt(discriminant)) / (2 * A);
                document.getElementById('kinematicResult4').innerText = `Possible Times (t): ${t1.toFixed(2)} s, ${t2.toFixed(2)} s`;
            } else {
                document.getElementById('kinematicResult4').innerText = 'No real solution for time with the given inputs.';
            }
        } else {
            document.getElementById('kinematicResult4').innerText = 'Please enter valid values for displacement, final velocity, and acceleration to calculate time.';
        }
    }
}
function calculateTension() {
    const mass = parseFloat(document.getElementById('massTension').value);
    const gravity = parseFloat(document.getElementById('gravityTension').value);
    const additionalForce = parseFloat(document.getElementById('additionalForce').value) || 0; // Default to 0 if not provided

    if (!isNaN(mass) && !isNaN(gravity)) {
        const tension = (mass * gravity) + additionalForce;
        document.getElementById('tensionResult').innerText = `Tension (T): ${tension.toFixed(2)} N`;
    } else {
        document.getElementById('tensionResult').innerText = 'Please enter valid values for mass and gravity.';
    }
}
