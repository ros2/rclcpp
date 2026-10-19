Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

Contributors must sign-off each commit by adding a `Signed-off-by: ...`
line to commit messages to certify that they have the right to submit
the code they are contributing to the project according to the
[Developer Certificate of Origin (DCO)](https://developercertificate.org/).

Before opening a PR, read the ROS 2
[Making a PR](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Making-a-PR.html)
and [Developer Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Developer-Guide.html).
Build the change and run the tests and linters locally, and fill in the whole PR template.
PRs that do not use the template may be closed.

## AI-assisted contributions

AI-assisted contributions must follow the
[Guidelines for AI-assisted contributions in the ROS project](https://discourse.openrobotics.org/t/guidelines-for-ai-assisted-contributions-in-the-ros-project/55903).
In short:

* You are responsible for every line you submit and must be able to explain and defend it.
* Write PR descriptions and review replies yourself.
* Disclose material AI use in the PR template and with an `Assisted-by` commit trailer.
* Only list tests you actually ran.
* PRs with hallucinated APIs or unverified LLM output are closed without review, and repeat offenders may be blocked.

AI agents working in this repository should read [AGENTS.md](AGENTS.md).
