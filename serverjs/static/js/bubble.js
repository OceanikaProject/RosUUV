const allRanges = document.querySelectorAll(".form-group-range");
allRanges.forEach(wrap => {
    const numberInput = wrap.querySelector(".form-range");
    const bubble = wrap.querySelector(".bubble");
    const rangeInput = wrap.querySelector(".range-number")

//    range.addEventListener("input", () => {
//        setBubble(range, bubble);
//    });

    rangeInput.addEventListener("keypress", e => {
        if (e.which < 48 || e.which > 57) {
            e.preventDefault();
        }
    });

    rangeInput.addEventListener("input", e => {
        setRangeNumber(numberInput, e.currentTarget.value);
    });

    numberInput.addEventListener("input", e => {
        setRangeInput(rangeInput, e.currentTarget.value);
    });


//    setBubble(range, bubble);
});


function setBubble(range, bubble) {
    const val = range.value;
    const min = range.min ? range.min : 0;
    const max = range.max ? range.max : 1;
    const newVal = Number(((val - min) * 1) / (max - min));
    bubble.innerHTML = val;

    bubble.style.left = `calc(${newVal * 95.0}% + ${(6.5 * (1 - newVal))}px)`;
}

function setRangeNumber(range, value) {
    range.value = value;
}

function setRangeInput(number, value) {
    number.value = value;
}