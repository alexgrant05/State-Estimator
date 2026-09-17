// Clock-free KR260 Pmod output diagnostic.
module led_static_high (
    output logic led_0
);
    always_comb begin
        led_0 = 1'b1;
    end
endmodule
