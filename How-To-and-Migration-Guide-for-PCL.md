## How-To and Migration Guide for PCL

### Context

Every piece of code evolves over time. Documentation is written at a particular point in time and as the underlying code evolves, sooner or later some documentation will become outdated and no longer be applicable or simply not represent the current "best practices". Keeping documentation up-to-date ensures a positive experience for the library's users and good quality documentation frees contributors to spend time on improving on the actual code itself. 

Unfortunately, handling this task requires some prior knowledge of how the library evolved and what are the current best practices e.g.: how to use PCL in consumer projects. The mentors will need to actively intervene here to ensure best practices are followed.

### Goals
All items would include Tutorials, Guides and Examples:

* Touch up existing documentation to latest features
  * Update examples/guides to reflect forward movement in PCL
  * Create list of what to follow in order to migrate to newer API of PCL
* Walk Through documentation
  * Create example scenario which cover a single module exhaustively or show combined usage across different modules
* Identify holes in existing documentation
  * Identify functionality with high usage in community
  * Identify functionality with missing high level documentation
* Contribute new documentation

### Walk Through Documentation

Place yourself in the role of a potential new user of the library. What are the questions a new user will ask?

1. What sort of functionalities does this library provide?
2. How is the library organized and what's the role of each module/part/section?

Answering these questions will allow a user to decide if the library fulfills his/her needs. What's the next step?

1. How do I install this project on my machine?
  - Are there package distributions for my platform?
  - How do I compile it from source?
2. How do I incorporate PCL in my project? What's the bare minimum boiler plate code which allows the user to confirm that everything is working correctly?

From this point onward, the user will pick whatever functionality he/she requires and follow the corresponding tutorials to get things done.

### Touch up existing documentation
* Revisit and try the current existing tutorials. Do they make sense? Is there something not working or simply out of place?
* Find patterns in the changes made. Does the pattern make life of users easier?
  * Yes: it should be part of a migration guide
  * No: it's a UX regression. Create an issue and discuss how to remedy it with the community


### Identify holes in existing documentation
As you go through all of these items you will surely encounter situations that require changes because they are non-existent, incomplete, unclear, outdated or simply don't produce the expected results. Start going through those and submitting issues with proposed changes.

### Contribute new documentation
Use existing documentation as a spring board in case an existing feature is "close enough" to the new feature. Some example could be new algorithm for old thing.

Whenever in doubt open an issue in the issue tracker and present your case. Expose the problem in a detailed way and ask around for feedback. Remember to be specific about your questions and show us you that you understand what the problem is. If possible, always try to present one solution to the problem. The maintainers can always point you in the right direction but ultimately you're the one who will know the problem in depth and can validate if the suggested changes produce the desired outcome.