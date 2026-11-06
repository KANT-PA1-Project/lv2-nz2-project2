"""Process-local visible-ID confirmation; never a motion authorization.

A rejection clears the streak. Confirmation requires the same object ID
in at least two distinct increasing physics epochs in the current run.
This cannot prove global physical uniqueness of a repeated marker ID.
"""


class IdentityReobservation:
    def __init__(self,run_id,required=2):
        if not isinstance(run_id,str) or not run_id:raise ValueError('missing_identity_run')
        if type(required) is not int or required<2:raise ValueError('identity_confirmation_requires_two_epochs')
        self.run_id=run_id;self.required=required;self.last_step=-1
        self.identity=None;self.streak=0

    def reject(self):
        self.identity=None;self.streak=0
        return self.result()

    def confirm(self,identity,*,physics_step,run_id):
        if run_id!=self.run_id:
            self.reject();raise ValueError('identity_run_mismatch')
        if type(physics_step) is not int or physics_step<0 or physics_step<=self.last_step:
            self.reject();raise ValueError('identity_epoch_not_increasing')
        self.last_step=physics_step
        if type(identity) is not int or not 0<=identity<12:
            self.reject();raise ValueError('identity_is_not_registered_object')
        self.streak=self.streak+1 if identity==self.identity else 1
        self.identity=identity
        return self.result()

    def result(self):
        confirmed=self.streak>=self.required
        return dict(identity_confirmed=confirmed,identity_streak=self.streak,
                    reobserve_required=not confirmed,motion_authorized=False)
