class Callback(list):
    def __call__(self, *args) -> None:
        for listener in self:
            listener(*args)
