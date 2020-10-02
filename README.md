Library for sending packets across UART on MSP430, where the sending endpoint
might not be reliable (e.g.  might be running intermittently).

Modified for multiple uarts to run at once.

Further muddled with code that is specific to the artibeus satellite, in the
future we'll tease this out separately, but for now we need code dumped into the
ISR.
