
# The fastMavlink Library: Test Suite #

The fastMavlink C library includes what could be the most comprehensive test suite available for MAVLink code.

This is a bold statement, I know, so let me explain. The key point to observe is that test does not equal test. For sure, any test is better than no test, but tests can only be as good as they are written to be. In fact, by our very specie's nature, we are susceptible to certain fallacies, which tests do not necessarily alleviate, but tests easily trick us into believing in a level of confidence which is not factually established by the tests. The issues are largely two-fold. First, of course, one needs a proven reference, against which one can test. Second, we tend to write tests which test what we know, but not what we do not know.

## Some General Thoughts on Automated Test Suites

Examples for what I just said are plenty, I just want to mention two, in order to contrast with the approach I have taken here.

My first example are unit tests. I'm not an expert of unit tests, but the logic is simple. They work by feeding a certain set of inputs to a function and test if the results match the expectation. Obviously, the first issue is resolved since input and output are naturally defined. However, the second issue fully bites. Indeed, a beautyfull example has been given by [Catch2](https://github.com/catchorg/Catch2) in their [tutorial](https://github.com/catchorg/Catch2/blob/devel/docs/tutorial.md). They consider a function for calculating the factorial, and initially had set up a test which checks the inputs 1, 2, 3, 10. However, said function was faulty for the input value 0. Clearly, it was initially not realized that the handling of 0 could be a critical point of the code, so it was not identified as an important test case, and was missed (to their credit, they frankly acknowledge this!). Of course, the very moment the issue with 0 was realized, not only a test for 0 was added, but also the function was corrected. So, the actual key here was NOT to have a test suite, but to realize in the first place that 0 is a case to consider. Once realized, the function was written such that it is working for that case too, and also the test suite was set up accordingly ... which however has become kind of obsolete since the function was written such as to pass the test! A beautyfull example of what I mean with "We tend to write tests which test what we know, but not what we do not know". 

This in my opinon is a general issue with unit tests. By their very nature they test only few inputs and thus easily can miss critical points.

My second example is the fuzzy testing of the pymavlink-mavgen parser [(1)](https://auterion.com/improving-the-security-of-px4-by-fuzz-testing-the-mavlink-c-parser/) [(2)](https://github.com/Auterion/mavlink-fuzz-testing). Here, the parser was subjected to randomized inputs. Using this, a known issue was detected, and the code was changed to correct for it. However, the implemented correction is faulty too!!! I of course can't say what has happened. Either the corrected code was not tested again or, what seems more likely to me, the critical point introduced by the correction was not realized and the test setup not designed to catch it. You certainly recall: "We tend to ..." :)

Now, since I'm a human being too, I'm obviously also vulnerable to the fallacies. What one can do is to spend the effort and design tests which can minimize the probability of overlooking possible critical points. Mathematicians would attempt to prove the working of a code, but I do not have these skills. So, my best bet is to run plenty of tests, each with a different, random input. This idea is of course as old as humankind, and not my invention, and fuzzy tests are just incarnations of it. The point however is that one has to do it, and not just brag about it, and do it in a way which hopefully catches iusses. I obviously can't claim to have succeded, I however believe I can claim that I tried.

## Weaknesses of the Pymavlink-mavgen Tests

Again, I emphasize, any test is better than no test, and the very fact that a pymavlink-mavgen test suite is existing is great. However, as also said, tests can be missleading and trick us. In fact, the pymavlink-mavgen test suite has IMHO some weaknesses. It is another example, and demonstration, of what I was saying before. I like to analyze it here to some extend, to set the stage, and of course to also substantiate my first statement in the above.

Points I'd like to mention are:

1. only one input per message
2. special input values do not occur, especially no zeros at payload end
3. not all message functions are tested
4. no reference data, only input
5. not tested on targets which the C code is much used on

The first point is directly obvious from going through `testsuite.h` for the dialects, and the implications are too. The seond point can also be checked by going through `testsuite.h`. I may have overlooked one, but I could not find a case where the test payload would have ended with a zero byte. Obviously, the generator for the test payloads does not or is very unlikely to produce zero value. The third point refers to that the `mavlink_msg_xxx_get_yyy()` functions are not tested. 

Point 4 means that the specified input data for the payload is packed into a message structure or buffer, which is then unpacked, and the resulting payload compared to the input payload. Thus, the tests are circular, they only check that the functions are self-consistent, but not that the resulting byte streams are correct. This logical flaw is elsewhere known as confirmation bias (proving that if A is true then B is true, and vice versa, does not prove that A and B are true).

Point 5 I find obvious, but seems to be largely ignored. However, code can (and does) have switches to adapt to different platforms. So testing it on one platform and then to assume it is also working on a different platform is logically no different to assuming that all functions were written correctly. If we could be sure about this, one wouldn't need tests in the first place. For pymavlink-mavgen I'm not aware of tests on STM32 chips. Runing tests on the smaller but frequently used STM32 versions is actually not as trivial as I initially thought since they tend to be very flash and RAM hungry. But tests with a small dialect run on e.g. bluepills, and they do the job.

<!--These points in fact have some sad effects. A crucial part of the MAVLink v2 protocol is to trim the payload off trailing zero bytes. Thus, not testing payloads with trailing zeros (= point 2) is nothing less than a mistake as obvioulsy relevant code parts are not covered. This may have been the reason why in the fuzzy test mentioned before the faulty code correction was not identified. In combination with point 3 another issue is missed: The pymavlink-mavgen C code doesn't zero-fill the payload in the message structure.-->


## FastMavlink Test Suite

The approach taken by fastMavlink directly follows from the above: 

First, it generates random inputs, and attention was payed to that special values appear sufficiently frequently. Especially, the random input generators were designed such that the value 0 is obtained with ca 20% and the special values -1 or UNIT_MAX with ca 10% probability. 

Second, the individual tests are run many times. It is no issue to run 100000 tests per message on a PC. 

Third, the fastMavlink library does not only test itself but is also tested against the pymavlink-mavgen library. That is, the pymavlink-mavgen library's results are taken as reference. The tests do not imply that both libraries behave exactly the same. They in fact do not, e.g., with respect to zero-byte filling. However, the tests imply that fastMavlink behaves exactly like pymavlink-mavgen with regards to the bytes on the wire and payload values - and this is what counts, right.

Lastly, the tests can be run on Windows PCs by virtue of Visual Studio projects and on STM32 chips by virtue of  Arduino sketches. (this is not yet fully available)

Several test suites exist:

- The `_messages` test suite tests the message functions, and the functions they call. For each message all functions in the respective `mavlink_msg_xxx.h` file are tested. Created message structures and frame buffers are validated against pymavlink-mavgen.

- The `_parser` test suite tests the parser functions for the messages of a dialect (usually all.xml). It generates a byte stream of N messages, which are randomly selected from the chosen dialect and filled with random header and payload. It validates the byte stream against pymavlink-mavgen. It then plays the byte stream to the two parsers available with fastMavlink, and checks the parsing result against the known sequence of messages.

- The `_parser_random` test suite tests the parser functions for random frames. It generates a byte stream of N frames, where the frames are created entirely randomly but such as to form valid MAVLink frames (i.e. correct len and checksum fields). It then plays the byte stream to fastMavlink's low-level parser, and checks the parsing result against the known sequence of frames, and messages. There is no testing against pymavlink-mavgen, it is thus circular.

## Limitations

First off, also these tests are written by a human being, and are hence limited by this fact.

There is no test of the parsers with faulty byte streams. (I'd like to subject the parser functions to an entirely random input stream, but have not yet figured out how to check and validate their behavior)

Lastly, I'd like to frankly admit that the tests are clearly not (yet) written as nicely as I would like to have them and as they could be; there is room for much improvement.





