const allRangeNumberInputs = document.querySelectorAll(".range-number")
allRangeNumberInputs.forEach(rangeNumber => {
    rangeNumber.addEventListener("change", (e) => {
        // alert(e.currentTarget.value);
        setRangeNumber(e.currentTarget.value);
    });
});

setRangeNumber = (number) => {
    console.log(number);
}